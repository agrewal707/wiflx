#ifndef WIFLX_SCRATCH_SC_FLEXFRAMESYNC_H
#define WIFLX_SCRATCH_SC_FLEXFRAMESYNC_H

#include <liquid/liquid.h>

typedef struct {
    unsigned int check;         // data validity check
    unsigned int fec0;          // forward error-correction scheme (inner)
    unsigned int fec1;          // forward error-correction scheme (outer)
    unsigned int mod_scheme;    // modulation scheme
} sc_flexframesyncprops_s;

typedef struct sc_flexframesync_s * sc_flexframesync;

// create sc_flexframesync object
//  _callback   :   callback function
//  _userdata   :   user data pointer passed to callback function
sc_flexframesync sc_flexframesync_create(framesync_callback _callback,
                                   void *             _userdata);

// destroy frame synchronizer
int sc_flexframesync_destroy(sc_flexframesync _q);

// print frame synchronizer internal properties
int sc_flexframesync_print(sc_flexframesync _q);

// reset frame synchronizer internal state
int sc_flexframesync_reset(sc_flexframesync _q);

// has frame been detected?
int sc_flexframesync_is_frame_open(sc_flexframesync _q);

// change length of user-defined region in header
int sc_flexframesync_set_header_len(sc_flexframesync _q,
                                 unsigned int  _len);

// enable or disable soft decoding of header
int sc_flexframesync_decode_header_soft(sc_flexframesync _q,
                                     int           _soft);

// enable or disable soft decoding of payload
int sc_flexframesync_decode_payload_soft(sc_flexframesync _q,
                                      int           _soft);

// set properties for header section
int sc_flexframesync_set_header_props(sc_flexframesync          _q,
                                   sc_flexframesyncprops_s * _props);

// push samples through frame synchronizer
//  _q      :   frame synchronizer object
//  _x      :   input samples, [size: _n x 1]
//  _n      :   number of input samples
int sc_flexframesync_execute(sc_flexframesync          _q,
                          liquid_float_complex * _x,
                          unsigned int           _n);

// frame data statistics
int              sc_flexframesync_reset_framedatastats(sc_flexframesync _q);
framedatastats_s sc_flexframesync_get_framedatastats  (sc_flexframesync _q);

// enable/disable debugging
int sc_flexframesync_debug_enable(sc_flexframesync _q);
int sc_flexframesync_debug_disable(sc_flexframesync _q);
int sc_flexframesync_debug_print(sc_flexframesync _q,
                               const char *  _filename);

#endif // WIFLX_SCRATCH_SC_FLEXFRAMESYNC_H
