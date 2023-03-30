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
// sc_flexframegen.c
//
// flexible frame generator
//

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <assert.h>
#include <complex.h>

#include "sc_common.h"
#include "sc_flexframegen.h"

#define PACKETIZER_VERSION (1)
#define FLEXFRAME_PROTOCOL  (101+PACKETIZER_VERSION)
#define FLEXFRAME_H_DEC    (6) // decoded length

// reconfigure internal properties
int           sc_flexframegen_reconfigure      (sc_flexframegen _q);
float complex sc_flexframegen_generate_symbol  (sc_flexframegen _q);
float complex sc_flexframegen_generate_preamble(sc_flexframegen _q);
float complex sc_flexframegen_generate_header  (sc_flexframegen _q);
float complex sc_flexframegen_generate_payload (sc_flexframegen _q);
float complex sc_flexframegen_generate_tail    (sc_flexframegen _q);

// default sc_flexframegen properties
static sc_flexframegenprops_s sc_flexframegenprops_default = {
    LIQUID_CRC_16,      // check
    LIQUID_FEC_NONE,    // fec0
    LIQUID_FEC_NONE,    // fec1
    LIQUID_MODEM_QPSK,  // mod_scheme
};

static sc_flexframegenprops_s sc_flexframegenprops_header_default = {
    LIQUID_CRC_16,      // check
    LIQUID_FEC_NONE,    // fec0
    LIQUID_FEC_NONE,    // fec1
    LIQUID_MODEM_QPSK,  // mod_scheme
};

int sc_flexframegenprops_init_default(sc_flexframegenprops_s * _props)
{
    memmove(_props, &sc_flexframegenprops_default, sizeof(sc_flexframegenprops_s));
    return LIQUID_OK;
}

struct sc_flexframegen_s {
    // interpolator
    unsigned int    k;                  // interp samples/symbol (fixed at 2)
    unsigned int    m;                  // interp filter delay (symbols)
    float           beta;               // excess bandwidth factor
    firinterp_crcf  interp;             // interpolator object
    float complex   buf_interp[2];      // output interpolator buffer [size: k x 1]

    sc_flexframegenprops_s props;          // payload properties
    sc_flexframegenprops_s header_props;   // header properties

    // preamble
    float complex * preamble_pn;        // p/n sequence

    // header
    unsigned char * header;             // header data
    unsigned int    header_user_len;    // header user section length
    unsigned int    header_dec_len;     // header length (decoded)
    qpacketmodem    header_encoder;     // header encoder/modulator
    unsigned int    header_mod_len;     // header length (encoded/modulated)
    float complex * header_mod;         // header symbols (encoded/modulated)
    qpilotgen       header_pilotgen;    // header pilot symbol generator
    unsigned int    header_sym_len;     // header length (pilots added)
    float complex * header_sym;         // header symbols (pilots added)

    // payload

    unsigned int    payload_dec_len;    // length of decoded
    qpacketmodem    payload_encoder;    // packet encoder/modulator
    unsigned int    payload_mod_len;    // payload length (encoded/modulated)
    float complex * payload_mod;        // payload symbols (encoded/modulated)
    qpilotgen       payload_pilotgen;   // payload pilot symbol generator
    unsigned int    payload_sym_len;    // length of encoded/modulated payload
    float complex * payload_sym;        // encoded payload symbols

    // counters/states
    unsigned int    symbol_counter;     // output symbol number
    unsigned int    sample_counter;     // output sample number
    int             frame_assembled;    // frame assembled flag
    int             frame_complete;     // frame completed flag
    enum {
                    STATE_PREAMBLE=0,   // write preamble p/n sequence
                    STATE_HEADER,       // write header symbols
                    STATE_PAYLOAD,      // write payload symbols
                    STATE_TAIL,         // tail symbols
    }               state;              // write state
};

sc_flexframegen sc_flexframegen_create(sc_flexframegenprops_s * _fgprops)
{
    sc_flexframegen q = (sc_flexframegen) malloc(sizeof(struct sc_flexframegen_s));
    unsigned int i;

    // create pulse-shaping filter
    q->k      = 2;
    q->m      = 7;
    q->beta   = 0.2f;
    //q->interp = firinterp_crcf_create_prototype(LIQUID_FIRFILT_ARKAISER,q->k,q->m,q->beta,0);
    q->interp = firinterp_crcf_create_prototype(LIQUID_FIRFILT_RRC,q->k,q->m,q->beta,0);

    // generate pn sequence
    q->preamble_pn = (float complex *) malloc(64*sizeof(float complex));
    msequence ms = msequence_create(7, 0x0089, 1);
    for (i=0; i<64; i++) {
        q->preamble_pn[i] = (msequence_advance(ms) ? M_SQRT1_2 : -M_SQRT1_2);
        q->preamble_pn[i] += (msequence_advance(ms) ? M_SQRT1_2 : -M_SQRT1_2) * _Complex_I;
    }
    msequence_destroy(ms);

    // reset object
    sc_flexframegen_reset(q);

    // header encoder/modulator
    q->header = NULL;
    q->header_mod = NULL;
    q->header_sym = NULL;
    q->header_encoder = NULL;
    q->header_pilotgen = NULL;
    q->header_user_len = 0;

    // payload encoder/modulator
    q->payload_mod = NULL;
    q->payload_sym = NULL;
    q->payload_encoder = qpacketmodem_create();
    q->payload_dec_len = 64;
    q->payload_pilotgen = NULL;

    // set payload properties
    sc_flexframegen_setprops(q, _fgprops);
    sc_flexframegen_set_header_props(q, NULL);

    // return pointer to main object
    return q;
}

int sc_flexframegen_destroy(sc_flexframegen _q)
{
    // destroy internal objects
    firinterp_crcf_destroy(_q->interp);
    qpacketmodem_destroy  (_q->header_encoder);
    qpilotgen_destroy     (_q->header_pilotgen);
    qpacketmodem_destroy  (_q->payload_encoder);
    qpilotgen_destroy     (_q->payload_pilotgen);

    // free buffers/arrays
    free(_q->preamble_pn);  // preamble symbols
    free(_q->header);       // header bytes
    free(_q->header_mod);   // encoded/modulated header symbols 
    free(_q->header_sym);
    free(_q->payload_mod);  // encoded/modulated payload symbols
    free(_q->payload_sym);  //

    // destroy frame generator
    free(_q);
    return LIQUID_OK;
}

// print sc_flexframegen object internals
int sc_flexframegen_print(sc_flexframegen _q)
{
    unsigned int num_frame_symbols =
            64 +                    // preamble p/n sequence length
            _q->header_sym_len +    // header symbols
            _q->payload_sym_len +   // number of modulation symbols
            2*_q->m;                // number of tail symbols
    unsigned int num_frame_bits = 8*_q->payload_dec_len;
    float eta = (float)num_frame_bits / (float)num_frame_symbols;

    printf("sc_flexframegen:\n");
    printf("  head          : %u symbols\n", _q->m);
    printf("  preamble      : %u\n", 64);
    printf("  header        : %u symbols (%u bytes)\n", _q->header_sym_len, _q->header_dec_len);
    printf("  payload       : %u symbols (%u bytes)\n", _q->payload_sym_len, _q->payload_dec_len);
    printf("    payload crc : %s\n", crc_scheme_str[_q->props.check][1]);
    printf("    fec (inner) : %s\n", fec_scheme_str[_q->props.fec0][1]);
    printf("    fec (outer) : %s\n", fec_scheme_str[_q->props.fec1][1]);
    printf("    mod scheme  : %s\n", modulation_types[_q->props.mod_scheme].name);
    printf("  tail          : %u symbols\n", _q->m);
    printf("  total         : %u symbols\n", num_frame_symbols);
    printf("  efficiency    : %.2f bits/second/Hz\n", eta);
    return LIQUID_OK;
}

// reset sc_flexframegen object internals
int sc_flexframegen_reset(sc_flexframegen _q)
{
    // reset internal counters and state
    _q->symbol_counter  = 0;
    _q->sample_counter  = 0;
    _q->frame_assembled = 0;
    _q->frame_complete  = 0;
    _q->state           = STATE_PREAMBLE;
    return LIQUID_OK;
}

// is frame assembled?
int sc_flexframegen_is_assembled(sc_flexframegen _q)
{
    return _q->frame_assembled;
}

// get sc_flexframegen properties
//  _q      :   frame generator object
//  _props  :   frame generator properties structure pointer
int sc_flexframegen_getprops(sc_flexframegen          _q,
                           sc_flexframegenprops_s * _props)
{
    // copy properties structure to output pointer
    memmove(_props, &_q->props, sizeof(sc_flexframegenprops_s));
    return LIQUID_OK;
}

// set sc_flexframegen properties
//  _q      :   frame generator object
//  _props  :   frame generator properties structure pointer
int sc_flexframegen_setprops(sc_flexframegen          _q,
                          sc_flexframegenprops_s * _props)
{
    // if frame is already assembled, give warning
    if (_q->frame_assembled) {
        return sc_error(LIQUID_EICONFIG,"sc_flexframegen_setprops(), frame is already assembled; must reset() first");
        return -1;
    }

    // if properties object is NULL, initialize with defaults
    if (_props == NULL)
        return sc_flexframegen_setprops(_q, &sc_flexframegenprops_default);

    // validate input
    if (_props->check == LIQUID_CRC_UNKNOWN || _props->check >= LIQUID_CRC_NUM_SCHEMES)
        return sc_error(LIQUID_EICONFIG,"sc_flexframegen_setprops(), invalid/unsupported CRC scheme");
    if (_props->fec0 == LIQUID_FEC_UNKNOWN || _props->fec1 == LIQUID_FEC_UNKNOWN)
        return sc_error(LIQUID_EICONFIG,"sc_flexframegen_setprops(), invalid/unsupported FEC scheme");
    if (_props->mod_scheme == LIQUID_MODEM_UNKNOWN )
        return sc_error(LIQUID_EICONFIG,"sc_flexframegen_setprops(), invalid/unsupported modulation scheme");

    // TODO : determine if re-configuration is necessary

    // copy properties to internal structure
    memmove(&_q->props, _props, sizeof(sc_flexframegenprops_s));

    // reconfigure payload buffers (reallocate as necessary)
    return sc_flexframegen_reconfigure(_q);
}

int sc_flexframegen_set_header_len(sc_flexframegen   _q,
                                 unsigned int   _len)
{
    // if frame is already assembled, give warning
    if (_q->frame_assembled)
        return sc_error(LIQUID_EICONFIG,"sc_flexframegen_setprops(), frame is already assembled; must reset() first");

    _q->header_user_len = _len;
    _q->header_dec_len = FLEXFRAME_H_DEC + _q->header_user_len;
    _q->header     = (unsigned char *) realloc(_q->header, _q->header_dec_len*sizeof(unsigned char));

    if (_q->header_encoder) {
        qpacketmodem_destroy(_q->header_encoder);
    }
    _q->header_encoder = qpacketmodem_create();
    qpacketmodem_configure(_q->header_encoder,
                           _q->header_dec_len,
                           _q->header_props.check,
                           _q->header_props.fec0,
                           _q->header_props.fec1,
                           _q->header_props.mod_scheme);
    _q->header_mod_len = qpacketmodem_get_frame_len(_q->header_encoder);
    _q->header_mod     = (float complex *) realloc(_q->header_mod, _q->header_mod_len*sizeof(float complex));

    // create header pilot sequence generator
    if (_q->header_pilotgen) {
        qpilotgen_destroy(_q->header_pilotgen);
    }
    _q->header_pilotgen = qpilotgen_create(_q->header_mod_len, 16);
    _q->header_sym_len  = qpilotgen_get_frame_len(_q->header_pilotgen);
    _q->header_sym      = (float complex *) realloc(_q->header_sym, _q->header_sym_len*sizeof(float complex));
    //printf("header: %u bytes > %u mod > %u sym\n", 64, _q->header_mod_len, _q->header_sym_len);
    return LIQUID_OK;
}

int sc_flexframegen_set_header_props(sc_flexframegen          _q,
                                  sc_flexframegenprops_s * _props)
{
    // if frame is already assembled, give warning
    if (_q->frame_assembled)
        return sc_error(LIQUID_EICONFIG,"sc_flexframegen_set_header_props(), frame is already assembled; must reset() first");

    if (_props == NULL)
        _props = &sc_flexframegenprops_header_default;

    // validate input
    if (_props->check == LIQUID_CRC_UNKNOWN || _props->check >= LIQUID_CRC_NUM_SCHEMES)
        return sc_error(LIQUID_EIMODE,"sc_flexframegen_set_header_props(), invalid/unsupported CRC scheme\n");
    if (_props->fec0 == LIQUID_FEC_UNKNOWN || _props->fec1 == LIQUID_FEC_UNKNOWN)
        return sc_error(LIQUID_EIMODE,"sc_flexframegen_set_header_props(), invalid/unsupported FEC scheme\n");
    if (_props->mod_scheme == LIQUID_MODEM_UNKNOWN )
        return sc_error(LIQUID_EIMODE,"sc_flexframegen_set_header_props(), invalid/unsupported modulation scheme\n");

    // copy properties to internal structure
    memmove(&_q->header_props, _props, sizeof(sc_flexframegenprops_s));

    // reconfigure payload buffers (reallocate as necessary)
    return sc_flexframegen_set_header_len(_q, _q->header_user_len);
}

// get frame length (number of samples)
unsigned int sc_flexframegen_getframelen(sc_flexframegen _q)
{
    if (!_q->frame_assembled) {
        sc_error(LIQUID_EICONFIG,"sc_flexframegen_getframelen(), frame not assembled!");
        return 0;
    }
    unsigned int num_frame_symbols =
            64 +                    // preamble p/n sequence length
            _q->header_sym_len +    // header symbols
            _q->payload_sym_len +   // number of modulation symbols
            2*_q->m;                // number of tail symbols

    return num_frame_symbols*_q->k; // k samples/symbol
}

// exectue frame generator (create the frame)
//  _q              :   frame generator object
//  _header         :   user-defined header
//  _payload        :   variable payload buffer (configured by setprops method)
//  _payload_dec_len:   length of payload
int sc_flexframegen_assemble(sc_flexframegen          _q,
                          const unsigned char * _header,
                          const unsigned char * _payload,
                          unsigned int          _payload_dec_len)
{
    // reset object
    sc_flexframegen_reset(_q);

    // reset interpolator
    firinterp_crcf_reset(_q->interp);

    // set decoded payload length
    _q->payload_dec_len = _payload_dec_len;

    // copy user-defined header to internal
    if (_header == NULL)
        memset(_q->header, 0x00, _q->header_user_len*sizeof(unsigned char));
    else
        memmove(_q->header, _header, _q->header_user_len*sizeof(unsigned char));

    // first several bytes of header are user-defined
    unsigned int n = _q->header_user_len;

    // add FLEXFRAME_PROTOCOL
    _q->header[n+0] = FLEXFRAME_PROTOCOL;

    // add payload length
    _q->header[n+1] = (_q->payload_dec_len >> 8) & 0xff;
    _q->header[n+2] = (_q->payload_dec_len     ) & 0xff;

    // add modulation scheme/depth (pack into single byte)
    _q->header[n+3]  = (unsigned int)(_q->props.mod_scheme);

    // add CRC, forward error-correction schemes
    //  CRC     : most-significant  3 bits of [n+4]
    //  fec0    : least-significant 5 bits of [n+4]
    //  fec1    : least-significant 5 bits of [n+5]
    _q->header[n+4]  = (_q->props.check & 0x07) << 5;
    _q->header[n+4] |= (_q->props.fec0) & 0x1f;
    _q->header[n+5]  = (_q->props.fec1) & 0x1f;

    // encode/modulate header
    qpacketmodem_encode(_q->header_encoder, _q->header, _q->header_mod);

    // add pilots
    qpilotgen_execute(_q->header_pilotgen, _q->header_mod, _q->header_sym);

    // reconfigure payload
    sc_flexframegen_reconfigure(_q);

    // encode/modulate payload
    qpacketmodem_encode(_q->payload_encoder, _payload, _q->payload_mod);

    // add pilots
    qpilotgen_execute(_q->payload_pilotgen, _q->payload_mod, _q->payload_sym);


    // set assembled flag
    _q->frame_assembled = 1;

    return LIQUID_OK;
}

// write samples of assembled frame, two samples at a time, returning
// '1' when frame is complete, '0' otherwise. Zeros will be written
// to the buffer if the frame is not assembled
//  _q          :   frame generator object
//  _buffer     :   output buffer [size: _buffer_len x 1]
//  _buffer_len :   output buffer length
int sc_flexframegen_write_samples(sc_flexframegen    _q,
                               float complex * _buffer,
                               unsigned int    _buffer_len)
{
    unsigned int i;
    for (i=0; i<_buffer_len; i++) {
        // determine if new sample needs to be written
        if (_q->sample_counter == 0) {
            // generate new symbol
            float complex sym = sc_flexframegen_generate_symbol(_q);

            // interpolate result
            firinterp_crcf_execute(_q->interp, sym, _q->buf_interp);
        }
        
        // write output sample from interpolator buffer
        _buffer[i] = _q->buf_interp[_q->sample_counter];
            
        // adjust sample counter
        _q->sample_counter = (_q->sample_counter + 1) % _q->k;
    }

    return _q->frame_complete;
}

//
// internal
//

// reconfigure internal buffers, objects, etc.
int sc_flexframegen_reconfigure(sc_flexframegen _q)
{
    // configure payload encoder/modulator
    qpacketmodem_configure(_q->payload_encoder,
                           _q->payload_dec_len,
                           _q->props.check,
                           _q->props.fec0,
                           _q->props.fec1,
                           _q->props.mod_scheme);

    // re-allocate memory for encoded message
    _q->payload_mod_len = qpacketmodem_get_frame_len(_q->payload_encoder);
    _q->payload_mod = (float complex*) realloc(_q->payload_mod,
                                               _q->payload_mod_len*sizeof(float complex));
    // ensure payload was reallocated appropriately
    if (_q->payload_mod == NULL)
        return sc_error(LIQUID_EIMEM,"sc_flexframegen_reconfigure(), could not re-allocate payload array");

    // create payload pilot sequence generator
    if (_q->payload_pilotgen) {
        qpilotgen_destroy(_q->payload_pilotgen);
    }
    _q->payload_pilotgen = qpilotgen_create(_q->payload_mod_len, 16);
    _q->payload_sym_len  = qpilotgen_get_frame_len(_q->payload_pilotgen);
    _q->payload_sym      = (float complex *) realloc(_q->payload_sym, _q->payload_sym_len*sizeof(float complex));
    //printf("payload: %u bytes > %u mod > %u sym\n", 64, _q->payload_mod_len, _q->payload_sym_len);

    return LIQUID_OK;
}

// fill interpolator buffer
float complex sc_flexframegen_generate_symbol(sc_flexframegen _q)
{
    // write zeros to buffer if frame is not assembled
    if (!_q->frame_assembled)
        return 0.0f;

    switch (_q->state) {
    case STATE_PREAMBLE: return sc_flexframegen_generate_preamble(_q);
    case STATE_HEADER:   return sc_flexframegen_generate_header  (_q);
    case STATE_PAYLOAD:  return sc_flexframegen_generate_payload (_q);
    case STATE_TAIL:     return sc_flexframegen_generate_tail    (_q);
    default:
        sc_error(LIQUID_EICONFIG,"sc_flexframegen_generate_symbol(), unknown/unsupported internal state");
    }

    return 0.0f;
}

// generate preamble
float complex sc_flexframegen_generate_preamble(sc_flexframegen _q)
{
    float complex symbol = _q->preamble_pn[_q->symbol_counter++];

    // check state
    if (_q->symbol_counter == 64) {
        _q->symbol_counter = 0;
        _q->state = STATE_HEADER;
    }
    return symbol;
}

// generate header
float complex sc_flexframegen_generate_header(sc_flexframegen _q)
{
    float complex symbol = _q->header_sym[_q->symbol_counter++];

    // check state
    if (_q->symbol_counter == _q->header_sym_len) {
        _q->symbol_counter = 0;
        _q->state = STATE_PAYLOAD;
    }
    return symbol;
}

// generate payload
float complex sc_flexframegen_generate_payload(sc_flexframegen _q)
{
    float complex symbol = _q->payload_sym[_q->symbol_counter++];

    // check state
    if (_q->symbol_counter == _q->payload_sym_len) {
        _q->symbol_counter = 0;
        _q->state = STATE_TAIL;
    }
    return symbol;
}

// generate tail
float complex sc_flexframegen_generate_tail(sc_flexframegen _q)
{
    // increment symbol counter
    _q->symbol_counter++;

    // check state
    if (_q->symbol_counter == 2*_q->m) {
        _q->symbol_counter  = 0;
        _q->frame_complete  = 1;
        _q->frame_assembled = 0;
    }

    return 0.0f;
}
