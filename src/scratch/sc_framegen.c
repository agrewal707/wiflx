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

//
// sc_framegen.c
//
// frame64 generator: 8-byte header, 64-byte payload, 1340-sample frame
//

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <assert.h>
#include <complex.h>

#include "sc_framegen.h"

struct sc_framegen_s {
    qpacketmodem    enc;                // packet encoder/modulator
    qpilotgen       pilotgen;           // pilot symbol generator
    float complex   pn_sequence[64];    // 64-symbol p/n sequence
    unsigned char   payload_dec[150];   // 600 = 150 bytes * 8 bits/bytes / 2 bits/symbol
    float complex   payload_sym[600];   // modulated payload symbols
    float complex   payload_tx[630];    // modulated payload symbols with pilots
    unsigned int    m;                  // filter delay (symbols)
    float           beta;               // filter excess bandwidth factor
    unsigned int    k;                  // samples per symbol
    firinterp_crcf  interp;             // pulse-shaping filter/interpolator
};

// create sc_framegen object
sc_framegen sc_framegen_create()
{
    sc_framegen q = (sc_framegen) malloc(sizeof(struct sc_framegen_s));
    q->m    = 7;
    q->beta = 0.2f;
    q->k = 2;

    unsigned int i;

    // generate pn sequence
    msequence ms = msequence_create(7, 0x0089, 1);
    for (i=0; i<64; i++) {
        q->pn_sequence[i]  = (msequence_advance(ms) ? M_SQRT1_2 : -M_SQRT1_2);
        q->pn_sequence[i] += (msequence_advance(ms) ? M_SQRT1_2 : -M_SQRT1_2)*_Complex_I;
    }
    msequence_destroy(ms);

    // create payload encoder/modulator object
    int check      = LIQUID_CRC_24;
    int fec0       = LIQUID_FEC_NONE;
    int fec1       = LIQUID_FEC_GOLAY2412;
    int mod_scheme = LIQUID_MODEM_QPSK;
    q->enc         = qpacketmodem_create();
    qpacketmodem_configure(q->enc, 72, check, fec0, fec1, mod_scheme);
    //qpacketmodem_print(q->enc);
    assert( qpacketmodem_get_frame_len(q->enc)==600 );

    // create pilot generator
    q->pilotgen = qpilotgen_create(600, 21);
    assert( qpilotgen_get_frame_len(q->pilotgen)==630 );

    // create pulse-shaping filter (k=2)
    //q->interp = firinterp_crcf_create_prototype(LIQUID_FIRFILT_ARKAISER,q->k,q->m,q->beta,0);
    q->interp = firinterp_crcf_create_prototype(LIQUID_FIRFILT_RRC,q->k,q->m,q->beta,0);

    // return main object
    return q;
}

// copy object
sc_framegen sc_framegen_copy(sc_framegen q_orig)
{
    // validate input
    if (q_orig == NULL) {
        fprintf (stderr, "sc_framegen_copy(), object cannot be NULL");
        return NULL;
    }

    // as this object is stateless, we can really create a new one
    return sc_framegen_create();
}

// destroy sc_framegen object
int sc_framegen_destroy(sc_framegen _q)
{
    // destroy internal objects
    qpacketmodem_destroy(_q->enc);
    qpilotgen_destroy(_q->pilotgen);
    firinterp_crcf_destroy(_q->interp);

    // free main object memory
    free(_q);
    return LIQUID_OK;
}

// print sc_framegen object internals
int sc_framegen_print(sc_framegen _q)
{
    float eta = (float) (8*(64 + 8)) / (float) (WIFLX_SC_LIQUID_FRAME64_LEN/_q->k);
    printf("sc_framegen [m=%u, beta=%4.2f]:\n", _q->m, _q->beta);
    printf("  preamble/etc.\n");
    printf("    * ramp/up symbols       :   %3u\n", _q->m);
    printf("    * p/n symbols           :   %3u\n", 64);
    printf("    * ramp\\down symbols     :   %3u\n", _q->m);
    printf("    * zero padding          :   %3u\n", 12);
    printf("  payload\n");
#if 0
    printf("    * payload crc           :   %s\n", crc_scheme_str[_q->check][1]);
    printf("    * fec (inner)           :   %s\n", fec_scheme_str[_q->fec0][1]);
    printf("    * fec (outer)           :   %s\n", fec_scheme_str[_q->fec1][1]);
#endif
    printf("    * payload len, uncoded  :   %3u bytes\n", 64);
    printf("    * payload len, coded    :   %3u bytes\n", 150);
    printf("    * modulation scheme     :   %s\n", modulation_types[LIQUID_MODEM_QPSK].name);
    printf("    * payload symbols       :   %3u\n", 600);
    printf("    * pilot symbols         :   %3u\n", 30);
    printf("  summary\n");
    printf("    * total symbols         :   %3u\n", WIFLX_SC_LIQUID_FRAME64_LEN/_q->k);
    printf("    * spectral efficiency   :   %6.4f b/s/Hz\n", eta);
    return LIQUID_OK;
}

// execute frame generator (creates a frame)
//  _q          :   frame generator object
//  _header     :   8-byte header data, NULL for random
//  _payload    :   64-byte payload data, NULL for random
//  _frame      :   output frame samples [size: WIFLX_SC_LIQUID_FRAME64_LEN x 1]
int sc_framegen_execute(sc_framegen      _q,
                       unsigned char * _header,
                       unsigned char * _payload,
                       float complex * _frame,
                       unsigned int  * _frame_len)
{
    unsigned int i;

    // concatenate header and payload
    for (i=0; i<8; i++)
        _q->payload_dec[i] = _header==NULL ? rand() & 0xff : _header[i];
    for (i=0; i<64; i++)
        _q->payload_dec[i+8] = _payload==NULL ? rand() & 0xff : _payload[i];

    // run packet encoder and modulator
    qpacketmodem_encode(_q->enc, _q->payload_dec, _q->payload_sym);

    // add pilot symbols
    qpilotgen_execute(_q->pilotgen, _q->payload_sym, _q->payload_tx);

    unsigned int n=0;

    // reset interpolator
    firinterp_crcf_reset(_q->interp);

    // p/n sequence
    for (i=0; i<64; i++) {
        firinterp_crcf_execute(_q->interp, _q->pn_sequence[i], &_frame[n]);
        n+=_q->k;
    }

    // frame payload
    for (i=0; i<630; i++) {
        firinterp_crcf_execute(_q->interp, _q->payload_tx[i], &_frame[n]);
        n+=_q->k;
    }

    // interpolator settling
    for (i=0; i<_q->k*_q->m + 2 + 10; i++) {
        firinterp_crcf_execute(_q->interp, 0.0f, &_frame[n]);
        n+=_q->k;
    }

    *_frame_len = n;

    assert(n==WIFLX_SC_LIQUID_FRAME64_LEN);
    return LIQUID_OK;
}


