/*
 * Copyright (c) 2007 - 2022 Joseph Gaeddert
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

// basic frame synchronizer with 8 bytes header and 64 bytes payload

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <complex.h>
#include <assert.h>

#include "sc_qdetector_cccf.h"
#include "sc_framesync.h"

#define FRAMESYNC64_ENABLE_EQ       0

// push samples through detection stage
int sc_framesync_execute_seekpn(sc_framesync   _q,
                               float complex _x);

// step receiver mixer, matched filter, decimator
//  _q      :   frame synchronizer
//  _x      :   input sample
//  _y      :   output symbol
int sc_framesync_step(sc_framesync     _q,
                     float complex   _x,
                     float complex * _y);

// push samples through synchronizer, saving received p/n symbols
int sc_framesync_execute_rxpreamble(sc_framesync   _q,
                                   float complex _x);

// receive payload symbols
int sc_framesync_execute_rxpayload(sc_framesync   _q,
                                  float complex _x);

// export debugging based on return value
//  0   : do not write file
//  >0  : write specific number (hex)
//  -1  : number of packets detected
//  -2  : id using first 4 bytes of header
//  -3  : write with random extension
int sc_framesync_debug_export(sc_framesync _q, int _code);

// sc_framesync object structure
struct sc_framesync_s {
    // callback
    framesync_callback  callback;   // user-defined callback function
    void *              userdata;   // user-defined data structure
    framesyncstats_s    framesyncstats; // frame statistic object (synchronizer)
    framedatastats_s    framedatastats; // frame statistic object (packet statistics)

    // synchronizer objects
    unsigned int        m;          // filter delay (symbols)
    float               beta;       // filter excess bandwidth factor
    unsigned int        k;          // samples per symbol
    sc_qdetector_cccf   detector;   // pre-demod detector
    float               tau_hat;    // fractional timing offset estimate
    float               dphi_hat;   // carrier frequency offset estimate
    float               phi_hat;    // carrier phase offset estimate
    float               gamma_hat;  // channel gain estimate
    nco_crcf            mixer;      // coarse carrier frequency recovery

    // timing recovery objects, states
    firpfb_crcf         mf;         // matched filter decimator
    unsigned int        npfb;       // number of filters in symsync
    int                 mf_counter; // matched filter output timer
    unsigned int        pfb_index;  // filterbank index
#if FRAMESYNC64_ENABLE_EQ
    eqlms_cccf          equalizer;  // equalizer (trained on p/n sequence)
#endif

    // preamble
    float complex preamble_pn[64];  // known 64-symbol p/n sequence
    float complex preamble_rx[64];  // received p/n symbols
    
    // payload decoder
    float complex payload_rx [630]; // received payload symbols with pilots
    float complex payload_sym[600]; // received payload symbols
    unsigned char payload_dec[ 72]; // decoded payload bytes
    qpacketmodem  dec;              // packet demodulator/decoder
    qpilotsync    pilotsync;        // pilot extraction, carrier recovery
    int           payload_valid;    // did payload pass crc?
    
    // status variables
    enum {
        FRAMESYNC64_STATE_DETECTFRAME=0,    // detect frame (seek p/n sequence)
        FRAMESYNC64_STATE_RXPREAMBLE,       // receive p/n sequence
        FRAMESYNC64_STATE_RXPAYLOAD,        // receive payload data
    }            state;
    unsigned int preamble_counter;  // counter: num of p/n syms received
    unsigned int payload_counter;   // counter: num of payload syms received

    windowcf     buf_debug;             // debug: raw input samples
    char *       prefix;                // debug: filename prefix
    char *       filename;              // debug: filename buffer
    unsigned int num_files_exported;    // debug: number of files exported
};

// create sc_framesync object
//  _callback       :   callback function invoked when frame is received
//  _userdata       :   user-defined data object passed to callback
sc_framesync sc_framesync_create(framesync_callback _callback,
                               void *             _userdata)
{
    sc_framesync q = (sc_framesync) malloc(sizeof(struct sc_framesync_s));
    q->callback = _callback;
    q->userdata = _userdata;
    q->m        = 7;    // filter delay (symbols)
    q->beta     = 0.2f; // excess bandwidth factor
    q->k        = 2;

    unsigned int i;

    // generate p/n sequence
    msequence ms = msequence_create(7, 0x0089, 1);
    for (i=0; i<64; i++) {
        q->preamble_pn[i]  = (msequence_advance(ms) ? M_SQRT1_2 : -M_SQRT1_2);
        q->preamble_pn[i] += (msequence_advance(ms) ? M_SQRT1_2 : -M_SQRT1_2)*_Complex_I;
    }
    msequence_destroy(ms);

    // create frame detector
    //q->detector = sc_qdetector_cccf_create_linear(q->preamble_pn, 64, LIQUID_FIRFILT_ARKAISER, q->k, q->m, q->beta);
    q->detector = sc_qdetector_cccf_create_linear(q->preamble_pn, 64, LIQUID_FIRFILT_RRC, q->k, q->m, q->beta);
    sc_qdetector_cccf_set_threshold(q->detector, 0.5f);

    // create symbol timing recovery filters
    q->npfb = 128;  // number of filters in the bank
    //q->mf   = firpfb_crcf_create_rnyquist(LIQUID_FIRFILT_ARKAISER, q->npfb,q->k,q->m,q->beta);
    q->mf   = firpfb_crcf_create_rnyquist(LIQUID_FIRFILT_RRC, q->npfb,q->k,q->m,q->beta);

#if FRAMESYNC64_ENABLE_EQ
    // create equalizer
    unsigned int p = 3;
    q->equalizer = eqlms_cccf_create_lowpass(2*q->k*p+1, 0.4f);
    eqlms_cccf_set_bw(q->equalizer, 0.05f);
#endif

    // create down-coverters for carrier phase tracking
    q->mixer = nco_crcf_create(LIQUID_NCO);
    
    // create payload demodulator/decoder object
    int check      = LIQUID_CRC_24;
    int fec0       = LIQUID_FEC_NONE;
    int fec1       = LIQUID_FEC_GOLAY2412;
    int mod_scheme = LIQUID_MODEM_QPSK;
    q->dec         = qpacketmodem_create();
    qpacketmodem_configure(q->dec, 72, check, fec0, fec1, mod_scheme);
    //qpacketmodem_print(q->dec);
    assert( qpacketmodem_get_frame_len(q->dec)==600 );

    // create pilot synchronizer
    q->pilotsync   = qpilotsync_create(600, 21);
    assert( qpilotsync_get_frame_len(q->pilotsync)==630);
 
    // reset global data counters
    sc_framesync_reset_framedatastats(q);

    // set debugging fields
    q->buf_debug= windowcf_create(LIQUID_FRAME64_LEN);
    q->prefix   = NULL;
    q->filename = NULL;
    q->num_files_exported = 0;
    sc_framesync_set_prefix(q, "sc_framesync");

    // reset state and return
    sc_framesync_reset(q);
    return q;
}

// copy object
sc_framesync sc_framesync_copy(sc_framesync q_orig)
{
    // validate input
    if (q_orig == NULL) {
        fprintf (stderr, "sc_framesync_copy(), object cannot be NULL");
        return NULL;
    }

    // allocate memory for new object
    sc_framesync q_copy = (sc_framesync) malloc(sizeof(struct sc_framesync_s));

    // copy entire memory space over and overwrite values as needed
    memmove(q_copy, q_orig, sizeof(struct sc_framesync_s));

    // set callback and userdata fields
    q_copy->callback = q_orig->callback;
    q_copy->userdata = q_orig->userdata;

    // copy objects
    q_copy->detector = sc_qdetector_cccf_copy(q_orig->detector);
    q_copy->mixer    = nco_crcf_copy      (q_orig->mixer);
    q_copy->mf       = firpfb_crcf_copy   (q_orig->mf);
    q_copy->dec      = qpacketmodem_copy  (q_orig->dec);
    q_copy->pilotsync= qpilotsync_copy    (q_orig->pilotsync);
    q_copy->buf_debug= windowcf_copy      (q_orig->buf_debug);

    // set prefix value
    q_copy->prefix   = NULL;
    q_copy->filename = NULL;
    sc_framesync_set_prefix(q_copy, q_orig->prefix);

    return q_copy;
}

// destroy frame synchronizer object, freeing all internal memory
int sc_framesync_destroy(sc_framesync _q)
{
    // destroy synchronization objects
    sc_qdetector_cccf_destroy(_q->detector);   // frame detector
    firpfb_crcf_destroy   (_q->mf);         // matched filter
    nco_crcf_destroy      (_q->mixer);      // coarse NCO
    qpacketmodem_destroy  (_q->dec);        // payload demodulator
    qpilotsync_destroy    (_q->pilotsync);  // pilot synchronizer
#if FRAMESYNC64_ENABLE_EQ
    eqlms_cccf_destroy    (_q->equalizer);  // LMS equalizer
#endif
    windowcf_destroy(_q->buf_debug);

    // free main object memory
    free(_q);
    return LIQUID_OK;
}

// print frame synchronizer object internals
int sc_framesync_print(sc_framesync _q)
{
    printf("sc_framesync:\n");
    return framedatastats_print(&_q->framedatastats);
}

// reset frame synchronizer object
int sc_framesync_reset(sc_framesync _q)
{
    // reset binary pre-demod synchronizer
    sc_qdetector_cccf_reset(_q->detector);

    // reset carrier recovery objects
    nco_crcf_reset(_q->mixer);

    // reset symbol timing recovery state
    firpfb_crcf_reset(_q->mf);
        
    // reset state
    _q->state           = FRAMESYNC64_STATE_DETECTFRAME;
    _q->preamble_counter= 0;
    _q->payload_counter = 0;
    
    // reset frame statistics
    _q->framesyncstats.evm = 0.0f;

    return LIQUID_OK;
}

// set the callback function
int sc_framesync_set_callback(sc_framesync        _q,
                             framesync_callback _callback)
{
    _q->callback = _callback;
    return LIQUID_OK;
}

// set the user-defined data field (context)
int sc_framesync_set_userdata(sc_framesync _q,
                             void *      _userdata)
{
    _q->userdata = _userdata;
    return LIQUID_OK;
}

// execute frame synchronizer
//  _q     :   frame synchronizer object
//  _x      :   input sample array [size: _n x 1]
//  _n      :   number of input samples
int sc_framesync_execute(sc_framesync   _q,
                        float complex * _x,
                        unsigned int    _n)
{
    unsigned int i;
    for (i=0; i<_n; i++) {
        // push sample into debug buffer
        windowcf_push(_q->buf_debug, _x[i]);

        switch (_q->state) {
        case FRAMESYNC64_STATE_DETECTFRAME:
            // detect frame (look for p/n sequence)
            sc_framesync_execute_seekpn(_q, _x[i]);
            break;
        case FRAMESYNC64_STATE_RXPREAMBLE:
            // receive p/n sequence symbols
            sc_framesync_execute_rxpreamble(_q, _x[i]);
            break;
        case FRAMESYNC64_STATE_RXPAYLOAD:
            // receive payload symbols
            sc_framesync_execute_rxpayload(_q, _x[i]);
            break;
        default:
            fprintf (stderr,"sc_framesync_exeucte(), unknown/unsupported state");
            return LIQUID_EINT;
        }
    }
    return LIQUID_OK;
}

// 
// internal methods
//

// execute synchronizer, seeking p/n sequence
//  _q     :   frame synchronizer object
//  _x      :   input sample
//  _sym    :   demodulated symbol
int sc_framesync_execute_seekpn(sc_framesync   _q,
                                float complex _x)
{
    // push through pre-demod synchronizer
    float complex * v = sc_qdetector_cccf_execute(_q->detector, _x);

    // check if frame has been detected
    if (v != NULL) {
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


        printf("***** frame detected! tau-hat:%8.4f(%u/%u), dphi-hat:%8.4f, gamma:%8.2f dB\n",
                _q->tau_hat, _q->pfb_index, _q->npfb, _q->dphi_hat, 20*log10f(_q->gamma_hat));

        // output filter scale
        firpfb_crcf_set_scale(_q->mf, 1.0f / (_q->k * _q->gamma_hat));

        // set frequency/phase of mixer
        nco_crcf_set_frequency(_q->mixer, _q->dphi_hat);
        nco_crcf_set_phase    (_q->mixer, _q->phi_hat );

        // update state
        _q->state = FRAMESYNC64_STATE_RXPREAMBLE;

        // run buffered samples through synchronizer
        unsigned int buf_len = sc_qdetector_cccf_get_buf_len(_q->detector);
        sc_framesync_execute(_q, v, buf_len);
    }
    return LIQUID_OK;
}

// step receiver mixer, matched filter, decimator
//  _q      :   frame synchronizer
//  _x      :   input sample
//  _y      :   output symbol
int sc_framesync_step(sc_framesync     _q,
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

#if FRAMESYNC64_ENABLE_EQ
    // push sample through equalizer
    eqlms_cccf_push(_q->equalizer, v);
#endif

    // increment counter to determine if sample is available
    _q->mf_counter++;
    int sample_available = (_q->mf_counter >= _q->k-1) ? 1 : 0;
    
    // set output sample if available
    if (sample_available) {
#if FRAMESYNC64_ENABLE_EQ
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
int sc_framesync_execute_rxpreamble(sc_framesync   _q,
                                    float complex _x)
{
    // step synchronizer
    float complex mf_out = 0.0f;
    int sample_available = sc_framesync_step(_q, _x, &mf_out);

    // compute output if timeout
    if (sample_available) {

        // save output in p/n symbols buffer
#if FRAMESYNC64_ENABLE_EQ
        unsigned int delay = 2*_q->m + 3; // delay from matched filter and equalizer
#else
        unsigned int delay = 2*_q->m;     // delay from matched filter
#endif
        if (_q->preamble_counter >= delay) {
            unsigned int index = _q->preamble_counter-delay;

            _q->preamble_rx[index] = mf_out;
        
#if FRAMESYNC64_ENABLE_EQ
            // train equalizer
            eqlms_cccf_step(_q->equalizer, _q->preamble_pn[index], mf_out);
#endif
        }

        // update p/n counter
        _q->preamble_counter++;

        // update state
        if (_q->preamble_counter == 64 + delay)
            _q->state = FRAMESYNC64_STATE_RXPAYLOAD;
    }
    return LIQUID_OK;
}

// execute synchronizer, receiving payload
//  _q      :   frame synchronizer object
//  _x      :   input sample
//  _sym    :   demodulated symbol
int sc_framesync_execute_rxpayload(sc_framesync   _q,
                                  float complex _x)
{
    // step synchronizer
    float complex mf_out = 0.0f;
    int sample_available = sc_framesync_step(_q, _x, &mf_out);

    // compute output if timeout
    if (sample_available) {
        // save payload symbols (modem input/output)
        _q->payload_rx[_q->payload_counter] = mf_out;

        // increment counter
        _q->payload_counter++;

        if (_q->payload_counter == 630) {
            // recover data symbols from pilots
            qpilotsync_execute(_q->pilotsync, _q->payload_rx, _q->payload_sym);

            // decode payload
            _q->payload_valid = qpacketmodem_decode(_q->dec,
                                                    _q->payload_sym,
                                                    _q->payload_dec);

            // update statistics
            _q->framedatastats.num_frames_detected++;
            _q->framedatastats.num_headers_valid  += _q->payload_valid;
            _q->framedatastats.num_payloads_valid += _q->payload_valid;
            _q->framedatastats.num_bytes_received += _q->payload_valid ? 64 : 0;

            // invoke callback
            if (_q->callback != NULL) {
                // set framesyncstats internals
                _q->framesyncstats.evm           = qpacketmodem_get_demodulator_evm(_q->dec); //qpilotsync_get_evm(_q->pilotsync);
                _q->framesyncstats.rssi          = 20*log10f(_q->gamma_hat);
                _q->framesyncstats.cfo           = nco_crcf_get_frequency(_q->mixer);
                _q->framesyncstats.framesyms     = _q->payload_sym;
                _q->framesyncstats.num_framesyms = 600;
                _q->framesyncstats.mod_scheme    = LIQUID_MODEM_QPSK;
                _q->framesyncstats.mod_bps       = 2;
                _q->framesyncstats.check         = LIQUID_CRC_24;
                _q->framesyncstats.fec0          = LIQUID_FEC_NONE;
                _q->framesyncstats.fec1          = LIQUID_FEC_GOLAY2412;

                // invoke callback method
                int rc =
                _q->callback(&_q->payload_dec[0],   // header is first 8 bytes
                             _q->payload_valid,
                             &_q->payload_dec[8],   // payload is last 64 bytes
                             64,
                             _q->payload_valid,
                             _q->framesyncstats,
                             _q->userdata);

                // export debugging based on return value
                sc_framesync_debug_export(_q, rc);
            }

            // reset frame synchronizer
            return sc_framesync_reset(_q);
        }
    }
    return LIQUID_OK;
}

// get detection threshold
float sc_framesync_get_threshold(sc_framesync _q)
{
    return sc_qdetector_cccf_get_threshold(_q->detector);
}

// set detection threshold
int sc_framesync_set_threshold(sc_framesync _q,
                              float       _threshold)
{
    return sc_qdetector_cccf_set_threshold(_q->detector, _threshold);
}

// set prefix for exporting debugging files, default: "sc_framesync"
//  _q      : frame sync object
//  _prefix : string with valid file path
int sc_framesync_set_prefix(sc_framesync  _q,
                           const char * _prefix)
{
    // skip if input is NULL pointer
    if (_prefix == NULL)
        return LIQUID_OK;

    // sanity check
    unsigned int n = strlen(_prefix);
    if (n > 1<<14) {
        fprintf(stderr,"sc_framesync_set_prefix(), input string size exceeds reasonable limits");
        return LIQUID_EICONFIG;
    }

    // reallocate memory, copy input, and return
    _q->prefix   = (char*) realloc(_q->prefix,   n+ 1);
    _q->filename = (char*) realloc(_q->filename, n+15);
    memmove(_q->prefix, _prefix, n);
    _q->prefix[n] = '\0';
    return LIQUID_OK;
}

// get prefix for exporting debugging files
const char * sc_framesync_get_prefix(sc_framesync  _q)
{
    return (const char*) _q->prefix;
}

// get number of files exported
unsigned int sc_framesync_get_num_files_exported(sc_framesync  _q)
{
    return _q->num_files_exported;
}

// get name of last debugging file written
const char * sc_framesync_get_filename(sc_framesync  _q)
{
    return _q->num_files_exported == 0 ? NULL : (const char*) _q->filename;
}

// reset frame data statistics
int sc_framesync_reset_framedatastats(sc_framesync _q)
{
    return framedatastats_reset(&_q->framedatastats);
}

// retrieve frame data statistics
framedatastats_s sc_framesync_get_framedatastats(sc_framesync _q)
{
    return _q->framedatastats;
}

// export debugging samples to file
int sc_framesync_debug_export(sc_framesync _q,
                             int         _code)
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
        return fprintf(stderr, "sc_framesync_debug_export(), invalid return code %d", _code);
        return LIQUID_EICONFIG;
    }

    FILE * fid = fopen(_q->filename,"wb");
    if (fid == NULL) {
        fprintf (stderr, "sc_framesync_debug_export(), could not open %s for writing", _q->filename);
        return LIQUID_EIO;
    }

    // TODO: write file header?

    // write debug buffer
    float complex * rc;
    windowcf_read(_q->buf_debug, &rc);
    fwrite(rc, sizeof(float complex), LIQUID_FRAME64_LEN, fid);

    // export framesync stats
    //framesyncstats_export(_q->framesyncstats, fid);

    // export measured offsets
    fwrite(&(_q->tau_hat),  sizeof(float), 1, fid);
    fwrite(&(_q->dphi_hat), sizeof(float), 1, fid);
    fwrite(&(_q->phi_hat),  sizeof(float), 1, fid);
    fwrite(&(_q->gamma_hat),sizeof(float), 1, fid);
    fwrite(&(_q->framesyncstats.evm), sizeof(float), 1, fid);

    // export payload values
    fwrite(_q->payload_rx,  sizeof(float complex), 630, fid);
    fwrite(_q->payload_sym, sizeof(float complex), 600, fid);
    fwrite(_q->payload_dec, sizeof(unsigned char),  72, fid);

    fclose(fid);
    _q->num_files_exported++;
    printf("sc_framesync_debug_export(), results written to %s (%u total)\n",
        _q->filename, _q->num_files_exported);
    return LIQUID_OK;
}

