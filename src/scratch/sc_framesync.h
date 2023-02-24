#ifndef WIFLX_SCRATCH_SC_FRAMESYNC_H
#define WIFLX_SCRATCH_SC_FRAMESYNC_H

#include <liquid/liquid.h>

typedef struct sc_framesync_s * sc_framesync;

// create sc_framesync object
//  _callback   :   callback function
//  _userdata   :   user data pointer passed to callback function
sc_framesync sc_framesync_create(framesync_callback _callback,
                               void *             _userdata);

// copy object
sc_framesync sc_framesync_copy(sc_framesync _q);

// destroy frame synchronizer
int sc_framesync_destroy(sc_framesync _q);

// print frame synchronizer internal properties
int sc_framesync_print(sc_framesync _q);

// reset frame synchronizer internal state
int sc_framesync_reset(sc_framesync _q);

// set the callback and userdata fields
int sc_framesync_set_callback(sc_framesync _q, framesync_callback _callback);
int sc_framesync_set_userdata(sc_framesync _q, void *             _userdata);

// push samples through frame synchronizer
//  _q      :   frame synchronizer object
//  _x      :   input samples, [size: _n x 1]
//  _n      :   number of input samples
int sc_framesync_execute(sc_framesync            _q,
                        liquid_float_complex * _x,
                        unsigned int           _n);

// set prefix for exporting debugging files, default: "sc_framesync"
//  _q      : frame sync object
//  _prefix : string with valid file path
int sc_framesync_set_prefix(sc_framesync  _q,
                           const char * _prefix);

// get prefix for exporting debugging files
const char * sc_framesync_get_prefix(sc_framesync  _q);

// get number of files exported
unsigned int sc_framesync_get_num_files_exported(sc_framesync  _q);

// get name of last file written
const char * sc_framesync_get_filename(sc_framesync  _q);

// get/set detection threshold
float sc_framesync_get_threshold(sc_framesync _q);
int   sc_framesync_set_threshold(sc_framesync _q, float _threshold);

// frame data statistics
int              sc_framesync_reset_framedatastats(sc_framesync _q);
framedatastats_s sc_framesync_get_framedatastats  (sc_framesync _q);

#endif // WIFLX_SCRATCH_SC_FRAMESYNC_H

