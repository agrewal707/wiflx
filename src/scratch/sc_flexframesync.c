/*
 * Copyright (c) 2007 - 2020 Joseph Gaeddert
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

//
// sc_flexframesync.c
//
// basic frame synchronizer
//
#include <time.h>

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <complex.h>
#include <assert.h>

#include "sc_common.h"
#include "sc_qdetector_cccf.h"
#include "sc_flexframesync.h"

#define DEBUG_FLEXFRAMESYNC         1
#define DEBUG_FLEXFRAMESYNC_PRINT   1
#define DEBUG_BUFFER_LEN            (16384)

#define FLEXFRAMESYNC_ENABLE_EQ     0

#define PACKETIZER_VERSION (1)
#define FLEXFRAME_PROTOCOL (101+PACKETIZER_VERSION)
#define FLEXFRAME_H_DEC    (6) // decoded length
#define FLEXFRAME_PREAMBLE_LEN (64)

// push samples through detection stage
int sc_flexframesync_execute_seekpn(sc_flexframesync _q,
                                 float complex _x);

// step receiver mixer, matched filter, decimator
//  _q      :   frame synchronizer
//  _x      :   input sample
//  _y      :   output symbol
int sc_flexframesync_step(sc_flexframesync   _q,
                       float complex   _x,
                       float complex * _y);

// push samples through synchronizer, saving received p/n symbols
int sc_flexframesync_execute_rxpreamble(sc_flexframesync _q,
                                     float complex _x);

// decode header and reconfigure payload
int sc_flexframesync_decode_header(sc_flexframesync _q);

// receive header symbols
int sc_flexframesync_execute_rxheader(sc_flexframesync _q,
                                   float complex _x);

// receive payload symbols
int sc_flexframesync_execute_rxpayload(sc_flexframesync _q,
                                    float complex _x);

// export debugging based on return value
//  0   : do not write file
//  >0  : write specific number (hex)
//  -1  : number of packets detected
//  -2  : id using first 4 bytes of header
//  -3  : write with random extension
int sc_flexframesync_debug_export(sc_flexframesync _q, int _code);

static sc_flexframesyncprops_s sc_flexframesyncprops_header_default = {
    LIQUID_CRC_16,      // check
    LIQUID_FEC_NONE,    // fec0
    LIQUID_FEC_NONE,    // fec1
    LIQUID_MODEM_QPSK,  // mod_scheme
};

// sc_flexframesync object structure
struct sc_flexframesync_s {
    // callback
    framesync_callback  callback;       // user-defined callback function
    void *              userdata;       // user-defined data structure
    framesyncstats_s    framesyncstats; // frame statistic object (synchronizer)
    framedatastats_s    framedatastats; // frame statistic object (packet statistics)
    
    // synchronizer objects
    unsigned int    m;                  // filter delay (symbols)
    float           beta;               // filter excess bandwidth factor
    unsigned int    k;                  // samples per symbol
    sc_qdetector_cccf  detector;        // pre-demod detector
    float           tau_hat;            // fractional timing offset estimate
    float           dphi_hat;           // carrier frequency offset estimate
    float           phi_hat;            // carrier phase offset estimate
    float           gamma_hat;          // channel gain estimate
    nco_crcf        mixer;              // carrier frequency recovery (coarse)
    nco_crcf        pll;                // carrier frequency recovery (fine)

    // timing recovery objects, states
    firpfb_crcf     mf;                 // matched filter decimator
    unsigned int    npfb;               // number of filters in symsync
    int             mf_counter;         // matched filter output timer
    unsigned int    pfb_index;          // filterbank index
#if FLEXFRAMESYNC_ENABLE_EQ
    eqlms_cccf      equalizer;          // equalizer (trained on p/n sequence)
#endif

    // preamble
    float complex * preamble_pn;        // known 64-symbol p/n sequence
    float complex * preamble_rx;        // received p/n symbols
    
    // header
    int             header_soft;        // header performs soft demod
    float complex * header_sym;         // header symbols with pilots (received)
    unsigned int    header_sym_len;     // header symbols with pilots (length)
    qpilotsync      header_pilotsync;   // header demodulator/decoder
    float complex * header_mod;         // header symbols (received)
    unsigned int    header_mod_len;     // header symbols (length)
    qpacketmodem    header_decoder;     // header demodulator/decoder
    unsigned int    header_user_len;    // length of user-defined array
    unsigned int    header_dec_len;     // length of header (decoded)
    unsigned char * header_dec;         // header bytes (decoded)
    int             header_valid;       // header CRC flag

    sc_flexframesyncprops_s header_props;   // header properties

    // payload
    int             payload_soft;       // payload performs soft demod
    //modemcf         payload_demod;      // payload demod (for phase recovery only)
    qpilotsync      payload_pilotsync;  // payload demodulator/decoder
    float complex * payload_mod;        // payload symbols (received) with pilots
    unsigned int    payload_mod_len;    // payload symbols (length)
    float complex * payload_sym;        // payload symbols (received)
    unsigned int    payload_sym_len;    // payload symbols (length)
    qpacketmodem    payload_decoder;    // payload demodulator/decoder
    unsigned char * payload_dec;        // payload data (bytes)
    unsigned int    payload_dec_len;    // payload data (length)
    int             payload_valid;      // payload CRC flag
    
    // status variables
    unsigned int    preamble_counter;   // counter: num of p/n syms received
    unsigned int    symbol_counter;     // counter: num of symbols received
    enum {
        FLEXFRAMESYNC_STATE_DETECTFRAME=0,  // detect frame (seek p/n sequence)
        FLEXFRAMESYNC_STATE_RXPREAMBLE,     // receive p/n sequence
        FLEXFRAMESYNC_STATE_RXHEADER,       // receive header data
        FLEXFRAMESYNC_STATE_RXPAYLOAD,      // receive payload data
    }               state;                  // receiver state

#if DEBUG_FLEXFRAMESYNC
    windowcf     debug_x;                // debug: raw input samples
    int          debug_qdetector_flush;  // debug: flag to set if we are flushing detector
    char *       prefix;                // debug: filename prefix
    char *       filename;              // debug: filename buffer
    unsigned int num_files_exported;    // debug: number of files exported
#endif
};

#define BILLION 1000000000.0f

// create sc_flexframesync object
//  _callback       :   callback function invoked when frame is received
//  _userdata       :   user-defined data object passed to callback
sc_flexframesync sc_flexframesync_create(framesync_callback _callback,
                                   void *             _userdata)
{
    sc_flexframesync q = (sc_flexframesync) malloc(sizeof(struct sc_flexframesync_s));
    q->callback = _callback;
    q->userdata = _userdata;
    q->m        = 7;    // filter delay (symbols)
    q->beta     = 0.2f; // excess bandwidth factor
    q->k        = 2;

    unsigned int i;


    // generate p/n sequence
    q->preamble_pn = (float complex*) malloc(FLEXFRAME_PREAMBLE_LEN*sizeof(float complex));
    q->preamble_rx = (float complex*) malloc(FLEXFRAME_PREAMBLE_LEN*sizeof(float complex));
    msequence ms = msequence_create(7, 0x0089, 1);
    for (i=0; i<FLEXFRAME_PREAMBLE_LEN; i++) {
        q->preamble_pn[i] = (msequence_advance(ms) ? M_SQRT1_2 : -M_SQRT1_2);
        q->preamble_pn[i] += (msequence_advance(ms) ? M_SQRT1_2 : -M_SQRT1_2) * _Complex_I;
    }
    msequence_destroy(ms);

    //struct timespec start, end;
    //clock_gettime (CLOCK_MONOTONIC, &start);

    // create frame detector
    //q->detector = sc_qdetector_cccf_create_linear(q->preamble_pn, FLEXFRAME_PREAMBLE_LEN, LIQUID_FIRFILT_ARKAISER, k, q->m, q->beta);
    q->detector = sc_qdetector_cccf_create_linear(q->preamble_pn, FLEXFRAME_PREAMBLE_LEN, LIQUID_FIRFILT_RRC, q->k, q->m, q->beta);
    sc_qdetector_cccf_set_threshold(q->detector, 0.5f);

    //clock_gettime (CLOCK_MONOTONIC, &end);

    // create symbol timing recovery filters
    q->npfb = 128;   // number of filters in the bank
    //q->mf   = firpfb_crcf_create_rnyquist(LIQUID_FIRFILT_ARKAISER, q->npfb,k,q->m,q->beta);
    q->mf   = firpfb_crcf_create_rnyquist(LIQUID_FIRFILT_RRC, q->npfb, q->k, q->m, q->beta);

    //double diff = BILLION * (end.tv_sec - start.tv_sec) + end.tv_nsec - start.tv_nsec;
	  //fprintf(stderr, "elapsed time = %f nanoseconds\n", diff);

#if FLEXFRAMESYNC_ENABLE_EQ
    // create equalizer
    unsigned int p = 3;
    q->equalizer = eqlms_cccf_create_lowpass(2*q->k*p+1, 0.4f);
    eqlms_cccf_set_bw(q->equalizer, 0.05f);
#endif

    // create down-coverters for carrier phase tracking
    q->mixer = nco_crcf_create(LIQUID_NCO);

    q->pll   = nco_crcf_create(LIQUID_NCO);
    nco_crcf_pll_set_bandwidth(q->pll, 1e-4f); // very low bandwidth
    
    // header demodulator/decoder
    q->header_sym = NULL;
    q->header_mod = NULL;
    q->header_dec = NULL;
    q->header_pilotsync = NULL;
    q->header_decoder = NULL;
    q->header_user_len = 0;
    q->header_soft = 0;
    sc_flexframesync_set_header_props(q, NULL);

    // payload demodulator for phase recovery
    //q->payload_demod = modemcf_create(LIQUID_MODEM_QPSK);

    // create payload demodulator/decoder object
    q->payload_sym = NULL;
    q->payload_mod = NULL;
    q->payload_dec = NULL;
    q->payload_pilotsync = NULL;
    q->payload_decoder = qpacketmodem_create();
    q->payload_soft = 0;

    // reset global data counters
    sc_flexframesync_reset_framedatastats(q);

#if DEBUG_FLEXFRAMESYNC
    q->debug_x  = windowcf_create(DEBUG_BUFFER_LEN);
    q->debug_qdetector_flush = 0;
    q->prefix   = NULL;
    q->filename = NULL;
    q->num_files_exported = 0;
    sc_flexframesync_set_prefix(q, "sc_flexframesync");
#endif

    // reset state and return
    sc_flexframesync_reset(q);
    return q;
}

// destroy frame synchronizer object, freeing all internal memory
int sc_flexframesync_destroy(sc_flexframesync _q)
{
#if DEBUG_FLEXFRAMESYNC
    // clean up debug objects (if created)
    if (_q->debug_x)
        windowcf_destroy(_q->debug_x);
#endif

    // free allocated arrays
    free(_q->preamble_pn);
    free(_q->preamble_rx);
    free(_q->header_sym);
    free(_q->header_mod);
    free(_q->header_dec);
    free(_q->payload_sym);
    free(_q->payload_dec);

    // destroy synchronization objects
    qpilotsync_destroy    (_q->header_pilotsync); // header pilotsync
    qpacketmodem_destroy  (_q->header_decoder);   // header demodulator/decoder
    //modemcf_destroy         (_q->payload_demod);    // payload demodulator (for PLL)
    qpilotsync_destroy    (_q->payload_pilotsync); // payload pilotsync
    qpacketmodem_destroy  (_q->payload_decoder);  // payload demodulator/decoder
    sc_qdetector_cccf_destroy(_q->detector);         // frame detector
    firpfb_crcf_destroy   (_q->mf);               // matched filter
    nco_crcf_destroy      (_q->mixer);            // oscillator (coarse)
    nco_crcf_destroy      (_q->pll);              // oscillator (fine)
#if FLEXFRAMESYNC_ENABLE_EQ
    eqlms_cccf_destroy    (_q->equalizer);        // LMS equalizer
#endif

    // free main object memory
    free(_q);
    return LIQUID_OK;
}

// print frame synchronizer object internals
int sc_flexframesync_print(sc_flexframesync _q)
{
    printf("sc_flexframesync:\n");
    return framedatastats_print(&_q->framedatastats);
}

// reset frame synchronizer object
int sc_flexframesync_reset(sc_flexframesync _q)
{
    // reset binary pre-demod synchronizer
    sc_qdetector_cccf_reset(_q->detector);

    // reset carrier recovery objects
    nco_crcf_reset(_q->mixer);
    nco_crcf_reset(_q->pll);

    // reset symbol timing recovery state
    firpfb_crcf_reset(_q->mf);
        
    // reset state
    _q->state           = FLEXFRAMESYNC_STATE_DETECTFRAME;
    _q->preamble_counter= 0;
    _q->symbol_counter  = 0;
    
    // reset frame statistics
    _q->framesyncstats.evm = 0.0f;
    return LIQUID_OK;
}

int sc_flexframesync_is_frame_open(sc_flexframesync _q)
{
    return (_q->state == FLEXFRAMESYNC_STATE_DETECTFRAME) ? 0 : 1;
}

int sc_flexframesync_set_header_len(sc_flexframesync _q,
                                  unsigned int  _len)
{
    _q->header_user_len = _len;
    _q->header_dec_len = FLEXFRAME_H_DEC + _q->header_user_len;
    _q->header_dec     = (unsigned char *) realloc(_q->header_dec, _q->header_dec_len*sizeof(unsigned char));
    if (_q->header_decoder) {
        qpacketmodem_destroy(_q->header_decoder);
    }
    _q->header_decoder = qpacketmodem_create();
    qpacketmodem_configure(_q->header_decoder,
                           _q->header_dec_len,
                           _q->header_props.check,
                           _q->header_props.fec0,
                           _q->header_props.fec1,
                           _q->header_props.mod_scheme);
    _q->header_mod_len = qpacketmodem_get_frame_len(_q->header_decoder);
    _q->header_mod     = (float complex*) realloc(_q->header_mod, _q->header_mod_len*sizeof(float complex));

    // header pilot synchronizer
    if (_q->header_pilotsync) {
        qpilotsync_destroy(_q->header_pilotsync);
    }
    _q->header_pilotsync = qpilotsync_create(_q->header_mod_len, 16);
    _q->header_sym_len   = qpilotsync_get_frame_len(_q->header_pilotsync);
    _q->header_sym       = (float complex*) realloc(_q->header_sym, _q->header_sym_len*sizeof(float complex));
    return LIQUID_OK;
}

int sc_flexframesync_decode_header_soft(sc_flexframesync _q,
                                     int           _soft)
{
    _q->header_soft = _soft;
    return LIQUID_OK;
}

int sc_flexframesync_decode_payload_soft(sc_flexframesync _q,
                                      int           _soft)
{
    _q->payload_soft = _soft;
    return LIQUID_OK;
}

int sc_flexframesync_set_header_props(sc_flexframesync          _q,
                                   sc_flexframesyncprops_s * _props)
{
    if (_props == NULL)
        _props = &sc_flexframesyncprops_header_default;

    // validate input
    if (_props->check == LIQUID_CRC_UNKNOWN || _props->check >= LIQUID_CRC_NUM_SCHEMES)
        return sc_error(LIQUID_EICONFIG,"sc_flexframesync_set_header_props(), invalid/unsupported CRC scheme");
    if (_props->fec0 == LIQUID_FEC_UNKNOWN || _props->fec1 == LIQUID_FEC_UNKNOWN)
        return sc_error(LIQUID_EICONFIG,"sc_flexframesync_set_header_props(), invalid/unsupported FEC scheme");
    if (_props->mod_scheme == LIQUID_MODEM_UNKNOWN )
        return sc_error(LIQUID_EICONFIG,"sc_flexframesync_set_header_props(), invalid/unsupported modulation scheme");

    // copy properties to internal structure
    memmove(&_q->header_props, _props, sizeof(sc_flexframesyncprops_s));

    // reconfigure payload buffers (reallocate as necessary)
    return sc_flexframesync_set_header_len(_q, _q->header_user_len);
}

// execute frame synchronizer
//  _q  :   frame synchronizer object
//  _x  :   input sample array [size: _n x 1]
//  _n  :   number of input samples
int sc_flexframesync_execute(sc_flexframesync   _q,
                          float complex * _x,
                          unsigned int    _n)
{
    unsigned int i;
    for (i=0; i<_n; i++) {
#if DEBUG_FLEXFRAMESYNC
        // write samples to debug buffer
        // NOTE: the debug_qdetector_flush prevents samples from being written twice
        if (!_q->debug_qdetector_flush)
            windowcf_push(_q->debug_x, _x[i]);
#endif
        switch (_q->state) {
        case FLEXFRAMESYNC_STATE_DETECTFRAME:
            // detect frame (look for p/n sequence)
            sc_flexframesync_execute_seekpn(_q, _x[i]);
            break;
        case FLEXFRAMESYNC_STATE_RXPREAMBLE:
            // receive p/n sequence symbols
            sc_flexframesync_execute_rxpreamble(_q, _x[i]);
            break;
        case FLEXFRAMESYNC_STATE_RXHEADER:
            // receive header symbols
            sc_flexframesync_execute_rxheader(_q, _x[i]);
            break;
        case FLEXFRAMESYNC_STATE_RXPAYLOAD:
            // receive payload symbols
            sc_flexframesync_execute_rxpayload(_q, _x[i]);
            break;
        default:
            return sc_error(LIQUID_EINT,"sc_flexframesync_exeucte(), unknown/unsupported internal state");
        }
    }
    return LIQUID_OK;
}

// 
// internal methods
//

// execute synchronizer, seeking p/n sequence
//  _q      :   frame synchronizer object
//  _x      :   input sample
//  _sym    :   demodulated symbol
int sc_flexframesync_execute_seekpn(sc_flexframesync _q,
                                  float complex _x)
{
    // push through pre-demod synchronizer
    float complex * v = sc_qdetector_cccf_execute(_q->detector, _x);

    // check if frame has been detected
    if (v == NULL)
        return LIQUID_OK;

    // get estimates
    _q->tau_hat   = sc_qdetector_cccf_get_tau  (_q->detector);
    _q->gamma_hat = sc_qdetector_cccf_get_gamma(_q->detector);
    _q->dphi_hat  = sc_qdetector_cccf_get_dphi (_q->detector);
    _q->phi_hat   = sc_qdetector_cccf_get_phi  (_q->detector);

    // set appropriate filterbank index
#if 0
    if (_q->tau_hat > 0) {
        _q->pfb_index = (unsigned int)(      _q->tau_hat  * _q->npfb) % _q->npfb;
        _q->mf_counter = 0;
    } else {
        _q->pfb_index = (unsigned int)((1.0f+_q->tau_hat) * _q->npfb) % _q->npfb;
        _q->mf_counter = 1;
    }
#endif
    // set appropriate filterbank index
    _q->mf_counter = _q->k - 2;
    _q->pfb_index =  0;
    int index = (int)(_q->tau_hat * _q->npfb);
    if (index < 0) {
        _q->mf_counter++;
        index += _q->npfb;
    }
    _q->pfb_index = index;

#if DEBUG_FLEXFRAMESYNC_PRINT
    printf("***** frame detected! tau-hat:%8.4f(%u/%u), dphi-hat:%8.4f, gamma:%8.2f dB\n",
            _q->tau_hat, _q->pfb_index, _q->npfb, _q->dphi_hat, 20*log10f(_q->gamma_hat));
#endif

    // output filter scale
    firpfb_crcf_set_scale(_q->mf, 1.0f / (_q->k * _q->gamma_hat));

    // set frequency/phase of mixer
    nco_crcf_set_frequency(_q->mixer, _q->dphi_hat);
    nco_crcf_set_phase    (_q->mixer, _q->phi_hat );

    // update state
    _q->state = FLEXFRAMESYNC_STATE_RXPREAMBLE;

#if DEBUG_FLEXFRAMESYNC
    // the debug_qdetector_flush prevents samples from being written twice
    _q->debug_qdetector_flush = 1;
#endif
    // run buffered samples through synchronizer
    unsigned int buf_len = sc_qdetector_cccf_get_buf_len(_q->detector);
    sc_flexframesync_execute(_q, v, buf_len);
#if DEBUG_FLEXFRAMESYNC
    _q->debug_qdetector_flush = 0;
#endif
    return LIQUID_OK;
}

// step receiver mixer, matched filter, decimator
//  _q      :   frame synchronizer
//  _x      :   input sample
//  _y      :   output symbol
int sc_flexframesync_step(sc_flexframesync   _q,
                       float complex   _x,
                       float complex * _y)
{
    // mix sample down
    float complex v;
    nco_crcf_mix_down(_q->mixer, _x, &v);
    nco_crcf_step    (_q->mixer);
    
    // push sample into filterbank
    firpfb_crcf_push   (_q->mf, v);
    firpfb_crcf_execute(_q->mf, _q->pfb_index, &v);

#if FLEXFRAMESYNC_ENABLE_EQ
    // push sample through equalizer
    eqlms_cccf_push(_q->equalizer, v);
#endif

    // increment counter to determine if sample is available
    _q->mf_counter++;
    int sample_available = (_q->mf_counter >= _q->k-1) ? 1 : 0;
    
    // set output sample if available
    if (sample_available) {
#if FLEXFRAMESYNC_ENABLE_EQ
        // compute equalizer output
        eqlms_cccf_execute(_q->equalizer, &v);
#endif

        // set output
        *_y = v;

        // decrement counter by k samples/symbol
        _q->mf_counter -= _q->k;
    }

    // return flag
    return sample_available;
}

// execute synchronizer, receiving p/n sequence
//  _q     :   frame synchronizer object
//  _x      :   input sample
//  _sym    :   demodulated symbol
int sc_flexframesync_execute_rxpreamble(sc_flexframesync _q,
                                     float complex _x)
{
    // step synchronizer
    float complex mf_out = 0.0f;
    int sample_available = sc_flexframesync_step(_q, _x, &mf_out);

    // compute output if timeout
    if (sample_available) {

        // save output in p/n symbols buffer
#if FLEXFRAMESYNC_ENABLE_EQ
        unsigned int delay = 2*_q->m + 3; // delay from matched filter and equalizer
#else
        unsigned int delay = 2*_q->m;     // delay from matched filter
#endif
        if (_q->preamble_counter >= delay) {
            unsigned int index = _q->preamble_counter-delay;

            _q->preamble_rx[index] = mf_out;
        
#if FLEXFRAMESYNC_ENABLE_EQ
            // train equalizer
            eqlms_cccf_step(_q->equalizer, _q->preamble_pn[index], mf_out);
#endif
        }

        // update p/n counter
        _q->preamble_counter++;

        // update state
        if (_q->preamble_counter == FLEXFRAME_PREAMBLE_LEN + delay)
            _q->state = FLEXFRAMESYNC_STATE_RXHEADER;
    }
    return LIQUID_OK;
}

// execute synchronizer, receiving header
//  _q      :   frame synchronizer object
//  _x      :   input sample
//  _sym    :   demodulated symbol
int sc_flexframesync_execute_rxheader(sc_flexframesync _q,
                                   float complex _x)
{
    // step synchronizer
    float complex mf_out = 0.0f;
    int sample_available = sc_flexframesync_step(_q, _x, &mf_out);

    // compute output if timeout
    if (sample_available) {
        // save payload symbols (modem input/output)
        _q->header_sym[_q->symbol_counter] = mf_out;

        // increment counter
        _q->symbol_counter++;

        if (_q->symbol_counter == _q->header_sym_len) {
            // decode header
            sc_flexframesync_decode_header(_q);

            if (_q->header_valid) {
                // continue on to decoding payload
                _q->symbol_counter = 0;
                _q->state = FLEXFRAMESYNC_STATE_RXPAYLOAD;
                return LIQUID_OK;
            }

            // update statistics
            _q->framedatastats.num_frames_detected++;

            // header invalid: invoke callback
            if (_q->callback != NULL) {
                // set framestats internals
                _q->framesyncstats.evm           = 0.0f; //20*log10f(sqrtf(_q->framesyncstats.evm / 600));
                _q->framesyncstats.rssi          = 20*log10f(_q->gamma_hat);
                _q->framesyncstats.cfo           = nco_crcf_get_frequency(_q->mixer);
                _q->framesyncstats.framesyms     = NULL;
                _q->framesyncstats.num_framesyms = 0;
                _q->framesyncstats.mod_scheme    = LIQUID_MODEM_UNKNOWN;
                _q->framesyncstats.mod_bps       = 0;
                _q->framesyncstats.check         = LIQUID_CRC_UNKNOWN;
                _q->framesyncstats.fec0          = LIQUID_FEC_UNKNOWN;
                _q->framesyncstats.fec1          = LIQUID_FEC_UNKNOWN;

                // invoke callback method
                _q->callback(_q->header_dec,
                             _q->header_valid,
                             NULL,  // payload
                             0,     // payload length
                             0,     // payload valid,
                             _q->framesyncstats,
                             _q->userdata);
            }

            // reset frame synchronizer
            return sc_flexframesync_reset(_q);
        }
    }
    return LIQUID_OK;
}

// decode header
int sc_flexframesync_decode_header(sc_flexframesync _q)
{
    // recover data symbols from pilots
    qpilotsync_execute(_q->header_pilotsync, _q->header_sym, _q->header_mod);

    // decode payload
    if (_q->header_soft) {
        _q->header_valid = qpacketmodem_decode_soft(_q->header_decoder,
                                                    _q->header_mod,
                                                    _q->header_dec);
    } else {
        _q->header_valid = qpacketmodem_decode(_q->header_decoder,
                                               _q->header_mod,
                                               _q->header_dec);
    }

    if (!_q->header_valid)
        return LIQUID_OK;

    // set fine carrier frequency and phase
    float dphi_hat = qpilotsync_get_dphi(_q->header_pilotsync);
    float  phi_hat = qpilotsync_get_phi (_q->header_pilotsync);
    //printf("residual offset: dphi=%12.8f, phi=%12.8f\n", dphi_hat, phi_hat);

    nco_crcf_set_frequency(_q->pll, dphi_hat);
    nco_crcf_set_phase    (_q->pll, phi_hat + dphi_hat * _q->header_sym_len);

    // first several bytes of header are user-defined
    unsigned int n = _q->header_user_len;

    // first byte is for expansion/version validation
    unsigned int protocol = _q->header_dec[n+0];
    if (protocol != FLEXFRAME_PROTOCOL) {
        _q->header_valid = 0;
        return sc_error(LIQUID_EICONFIG,"sc_flexframesync_decode_header(), invalid framing protocol %u (expected %u)", protocol, FLEXFRAME_PROTOCOL);
    }

    // strip off payload length
    unsigned int payload_dec_len = (_q->header_dec[n+1] << 8) | (_q->header_dec[n+2]);
    _q->payload_dec_len = payload_dec_len;

    // strip off modulation scheme/depth
    unsigned int mod_scheme = _q->header_dec[n+3];

    // strip off CRC, forward error-correction schemes
    //  CRC     : most-significant 3 bits of [n+4]
    //  fec0    : least-significant 5 bits of [n+4]
    //  fec1    : least-significant 5 bits of [n+5]
    unsigned int check = (_q->header_dec[n+4] >> 5 ) & 0x07;
    unsigned int fec0  = (_q->header_dec[n+4]      ) & 0x1f;
    unsigned int fec1  = (_q->header_dec[n+5]      ) & 0x1f;

    // validate properties
    if (mod_scheme == 0 || mod_scheme >= LIQUID_MODEM_NUM_SCHEMES) {
        _q->header_valid = 0;
        return sc_error(LIQUID_EICONFIG,"sc_flexframesync_decode_header(), invalid modulation scheme");
    } else if (check == LIQUID_CRC_UNKNOWN || check >= LIQUID_CRC_NUM_SCHEMES) {
        _q->header_valid = 0;
        return sc_error(LIQUID_EICONFIG,"sc_flexframesync_decode_header(), decoded CRC exceeds available");
    } else if (fec0 == LIQUID_FEC_UNKNOWN || fec0 >= LIQUID_FEC_NUM_SCHEMES) {
        _q->header_valid = 0;
        return sc_error(LIQUID_EICONFIG,"sc_flexframesync_decode_header(), decoded FEC (inner) exceeds available");
    } else if (fec1 == LIQUID_FEC_UNKNOWN || fec1 >= LIQUID_FEC_NUM_SCHEMES) {
        _q->header_valid = 0;
        return sc_error(LIQUID_EICONFIG,"sc_flexframesync_decode_header(), decoded FEC (outer) exceeds available");
    }

    // re-create payload demodulator for phase-locked loop
    //_q->payload_demod = modemcf_recreate(_q->payload_demod, mod_scheme);

    // reconfigure payload demodulator/decoder
    qpacketmodem_configure(_q->payload_decoder,
                           payload_dec_len, check, fec0, fec1, mod_scheme);

    // set length appropriately
    _q->payload_mod_len = qpacketmodem_get_frame_len(_q->payload_decoder);

    // re-allocate buffers accordingly
    _q->payload_mod = (float complex*) realloc(_q->payload_mod, (_q->payload_mod_len)*sizeof(float complex));
    _q->payload_dec = (unsigned char*) realloc(_q->payload_dec, (_q->payload_dec_len)*sizeof(unsigned char));

    if (_q->payload_mod == NULL || _q->payload_dec == NULL) {
        _q->header_valid = 0;
        return sc_error(LIQUID_EIMEM,"sc_flexframesync_decode_header(), could not re-allocate payload arrays");
    }

    // payload pilot synchronizer
    if (_q->payload_pilotsync) {
        qpilotsync_destroy(_q->payload_pilotsync);
    }
    _q->payload_pilotsync = qpilotsync_create(_q->payload_mod_len, 16);
    _q->payload_sym_len   = qpilotsync_get_frame_len(_q->payload_pilotsync);
    _q->payload_sym       = (float complex*) realloc(_q->payload_sym, _q->payload_sym_len*sizeof(float complex));

#if DEBUG_FLEXFRAMESYNC_PRINT
    // print results
    printf("sc_flexframesync_decode_header():\n");
    printf("    header crc      : %s\n", _q->header_valid ? "pass" : "FAIL");
    printf("    check           : %s\n", crc_scheme_str[check][1]);
    printf("    fec (inner)     : %s\n", fec_scheme_str[fec0][1]);
    printf("    fec (outer)     : %s\n", fec_scheme_str[fec1][1]);
    printf("    mod scheme      : %s\n", modulation_types[mod_scheme].name);
    printf("    payload sym len : %u\n", _q->payload_sym_len);
    printf("    payload mod len : %u\n", _q->payload_mod_len);
    printf("    payload dec len : %u\n", _q->payload_dec_len);
    printf("    user data       :");
    unsigned int i;
    for (i=0; i<_q->header_user_len; i++)
        printf(" %.2x", _q->header_dec[i]);
    printf("\n");
#endif
    return LIQUID_OK;
}


// execute synchronizer, receiving payload
//  _q      :   frame synchronizer object
//  _x      :   input sample
//  _sym    :   demodulated symbol
int sc_flexframesync_execute_rxpayload(sc_flexframesync _q,
                                    float complex _x)
{
    // step synchronizer
    float complex mf_out = 0.0f;
    int sample_available = sc_flexframesync_step(_q, _x, &mf_out);

    // compute output if timeout
    if (sample_available) {

#if 0
        // TODO: clean this up
        // mix down with fine-tuned oscillator
        nco_crcf_mix_down(_q->pll, mf_out, &mf_out);
        // track phase, accumulate error-vector magnitude
        unsigned int sym;
        modemcf_demodulate(_q->payload_demod, mf_out, &sym);
        float phase_error = modemcf_get_demodulator_phase_error(_q->payload_demod);
        float evm         = modemcf_get_demodulator_evm        (_q->payload_demod);
        nco_crcf_pll_step(_q->pll, phase_error);
        nco_crcf_step(_q->pll);
        _q->framesyncstats.evm += evm*evm;
#endif

        // save payload symbols (modem input/output)
        _q->payload_sym[_q->symbol_counter] = mf_out;

        // increment counter
        _q->symbol_counter++;

        if (_q->symbol_counter == _q->payload_sym_len) {
            // recover data symbols from pilots
            qpilotsync_execute(_q->payload_pilotsync, _q->payload_sym, _q->payload_mod);

            // decode payload
            if (_q->payload_soft) {
                _q->payload_valid = qpacketmodem_decode_soft(_q->payload_decoder,
                                                             _q->payload_mod,
                                                             _q->payload_dec);
            } else {
                _q->payload_valid = qpacketmodem_decode(_q->payload_decoder,
                                                        _q->payload_mod,
                                                        _q->payload_dec);
            }

            // update statistics
            _q->framedatastats.num_frames_detected++;
            _q->framedatastats.num_headers_valid++;
            _q->framedatastats.num_payloads_valid += _q->payload_valid;
            _q->framedatastats.num_bytes_received += _q->payload_dec_len;

            // invoke callback
            if (_q->callback != NULL) {
                // set framestats internals
                int ms = qpacketmodem_get_modscheme(_q->payload_decoder);
                _q->framesyncstats.evm           = qpacketmodem_get_demodulator_evm(_q->payload_decoder);
                _q->framesyncstats.rssi          = 20*log10f(_q->gamma_hat);
                _q->framesyncstats.cfo           = nco_crcf_get_frequency(_q->mixer);
                _q->framesyncstats.framesyms     = _q->payload_sym;
                _q->framesyncstats.num_framesyms = _q->payload_sym_len;
                _q->framesyncstats.mod_scheme    = ms;
                _q->framesyncstats.mod_bps       = modulation_types[ms].bps;
                _q->framesyncstats.check         = qpacketmodem_get_crc(_q->payload_decoder);
                _q->framesyncstats.fec0          = qpacketmodem_get_fec0(_q->payload_decoder);
                _q->framesyncstats.fec1          = qpacketmodem_get_fec1(_q->payload_decoder);

                // invoke callback method
                int rc = _q->callback(_q->header_dec,
                             _q->header_valid,
                             _q->payload_dec,
                             _q->payload_dec_len,
                             _q->payload_valid,
                             _q->framesyncstats,
                             _q->userdata);

#if DEBUG_FLEXFRAMESYNC
                // export debugging based on return value
                sc_flexframesync_debug_export(_q, rc);
#endif
            }

            // reset frame synchronizer
            return sc_flexframesync_reset(_q);
        }
    }
    return LIQUID_OK;
}

// reset frame data statistics
int sc_flexframesync_reset_framedatastats(sc_flexframesync _q)
{
    return framedatastats_reset(&_q->framedatastats);
}

// retrieve frame data statistics
framedatastats_s sc_flexframesync_get_framedatastats(sc_flexframesync _q)
{
    return _q->framedatastats;
}

// set prefix for exporting debugging files, default: "sc_framesync"
//  _q      : frame sync object
//  _prefix : string with valid file path
int sc_flexframesync_set_prefix(sc_flexframesync  _q,
                                const char * _prefix)
{
    // skip if input is NULL pointer
    if (_prefix == NULL)
        return LIQUID_OK;

    // sanity check
    unsigned int n = strlen(_prefix);
    if (n > 1<<14) {
        fprintf(stderr,"sc_flexframesync_set_prefix(), input string size exceeds reasonable limits");
        return LIQUID_EICONFIG;
    }

    // reallocate memory, copy input, and return
    _q->prefix   = (char*) realloc(_q->prefix,   n+ 1);
    _q->filename = (char*) realloc(_q->filename, n+15);
    memmove(_q->prefix, _prefix, n);
    _q->prefix[n] = '\0';
    return LIQUID_OK;
}

// export debugging samples to file
int sc_flexframesync_debug_export(sc_flexframesync _q, int _code)
{
    // determine what to do based on callback return code
    if (_code == 0) {
        // do not export file
        return LIQUID_OK;
    } else if (_code > 0) {
        // user-defined value
        sprintf(_q->filename,"%s_u%.8x.dat", _q->prefix, _code);
    } else if (_code == -1) {
        // based on number of packets detected
        sprintf(_q->filename,"%s_n%.8x.dat", _q->prefix,
                _q->framedatastats.num_frames_detected);
    } else if (_code == -2) {
        // decoded header (first 4 bytes)
        sprintf(_q->filename,"%s_h", _q->prefix);
        char * p = _q->filename + strlen(_q->prefix) + 2;
        for (unsigned int i=0; i<4; i++) {
            sprintf(p,"%.2x", _q->payload_dec[i]);
            p += 2;
        }
        sprintf(p,".dat");
    } else if (_code == -3) {
        // random extension
        sprintf(_q->filename,"%s_r%.8x.dat", _q->prefix, rand() & 0xffffffff);
    } else {
        return fprintf(stderr, "sc_flexframesync_debug_export(), invalid return code %d", _code);
        return LIQUID_EICONFIG;
    }

    FILE * fid = fopen(_q->filename,"wb");
    if (fid == NULL) {
        fprintf (stderr, "sc_flexframesync_debug_export(), could not open %s for writing", _q->filename);
        return LIQUID_EIO;
    }

    // TODO: write file header?

    unsigned int num_frame_symbols =
            FLEXFRAME_PREAMBLE_LEN + // preamble p/n sequence length
            _q->header_sym_len +     // header symbols
            _q->payload_sym_len +    // number of modulation symbols
            2*_q->m;                 // number of tail symbols

    // only write received frame samples with 16 pre-frame start samples
    unsigned int x_len = num_frame_symbols*_q->k + 16; 

    // write debug buffer
    float complex * rc;
    windowcf_read(_q->debug_x, &rc);
    fwrite(&x_len, sizeof(unsigned int), 1, fid);
    fwrite(rc+DEBUG_BUFFER_LEN-x_len, sizeof(float complex), x_len, fid);

    // export framesync stats
    //framesyncstats_export(_q->framesyncstats, fid);

    // export measured offsets
    fwrite(&(_q->tau_hat),  sizeof(float), 1, fid);
    fwrite(&(_q->dphi_hat), sizeof(float), 1, fid);
    fwrite(&(_q->phi_hat),  sizeof(float), 1, fid);
    fwrite(&(_q->gamma_hat),sizeof(float), 1, fid);
    fwrite(&(_q->framesyncstats.evm), sizeof(float), 1, fid);

    // export payload values
    fwrite(&(_q->payload_sym_len), sizeof(unsigned int), 1, fid);
    fwrite(_q->payload_sym,  sizeof(float complex), _q->payload_sym_len, fid);
    fwrite(&(_q->payload_mod_len), sizeof(unsigned int), 1, fid);
    fwrite(_q->payload_mod, sizeof(float complex), _q->payload_mod_len, fid);
    fwrite(&(_q->payload_dec_len), sizeof(unsigned int), 1, fid);
    fwrite(_q->payload_dec, sizeof(unsigned char),  _q->payload_dec_len, fid);

    fclose(fid);
    _q->num_files_exported++;
    printf("sc_flexframesync_debug_export(), results written to %s (%u total)\n",
        _q->filename, _q->num_files_exported);
    return LIQUID_OK;
}
