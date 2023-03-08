#ifndef WIFLX_SCRATCH_SC_FRAMEGEN_H
#define WIFLX_SCRATCH_SC_FRAMEGEN_H

#include <liquid/liquid.h>

//
// Basic frame generator (64 bytes data payload)
//

// frame length in samples
#define WIFLX_SC_LIQUID_FRAME64_LEN (720*2)

typedef struct sc_framegen_s * sc_framegen;

// create frame generator
sc_framegen sc_framegen_create();

// copy object
sc_framegen sc_framegen_copy(sc_framegen _q);

// destroy frame generator
int sc_framegen_destroy(sc_framegen _q);

// print frame generator internal properties
int sc_framegen_print(sc_framegen _q);

// generate frame
//  _q          :   frame generator object
//  _header     :   8-byte header data, NULL for random
//  _payload    :   64-byte payload data, NULL for random
//  _frame      :   output frame samples, [size: LIQUID_FRAME64_LEN x 1]
int sc_framegen_execute(sc_framegen             _q,
                       unsigned char *        _header,
                       unsigned char *        _payload,
                       liquid_float_complex * _frame,
                       unsigned int  *        _frame_len);

#endif // WIFLX_SCRATCH_SC_FRAMEGEN_H
