#ifndef WIFLX_SCRATCH_SC_FLEXFRAMEGEN_H
#define WIFLX_SCRATCH_SC_FLEXFRAMEGEN_H

#include <liquid/liquid.h>

// frame generator
typedef struct {
    unsigned int check;         // data validity check
    unsigned int fec0;          // forward error-correction scheme (inner)
    unsigned int fec1;          // forward error-correction scheme (outer)
    unsigned int mod_scheme;    // modulation scheme
} sc_flexframegenprops_s;

int sc_flexframegenprops_init_default(sc_flexframegenprops_s * _fgprops);

typedef struct sc_flexframegen_s * sc_flexframegen;

// create sc_flexframegen object
//  _props  :   frame properties (modulation scheme, etc.)
sc_flexframegen sc_flexframegen_create(sc_flexframegenprops_s * _props);

// destroy sc_flexframegen object
int sc_flexframegen_destroy(sc_flexframegen _q);

// print sc_flexframegen object internals
int sc_flexframegen_print(sc_flexframegen _q);

// reset sc_flexframegen object internals
int sc_flexframegen_reset(sc_flexframegen _q);

// is frame assembled?
int sc_flexframegen_is_assembled(sc_flexframegen _q);

// get frame properties
int sc_flexframegen_getprops(sc_flexframegen _q, sc_flexframegenprops_s * _props);

// set frame properties
int sc_flexframegen_setprops(sc_flexframegen _q, sc_flexframegenprops_s * _props);

// set length of user-defined portion of header
int sc_flexframegen_set_header_len(sc_flexframegen _q, unsigned int _len);

// set properties for header section
int sc_flexframegen_set_header_props(sc_flexframegen          _q,
                                  sc_flexframegenprops_s * _props);

// get length of assembled frame (samples)
unsigned int sc_flexframegen_getframelen(sc_flexframegen _q);

// assemble a frame from an array of data
//  _q              :   frame generator object
//  _header         :   frame header
//  _payload        :   payload data, [size: _payload_len x 1]
//  _payload_len    :   payload data length
int sc_flexframegen_assemble(sc_flexframegen          _q,
                          const unsigned char * _header,
                          const unsigned char * _payload,
                          unsigned int          _payload_len);

// write samples of assembled frame, two samples at a time, returning
// '1' when frame is complete, '0' otherwise. Zeros will be written
// to the buffer if the frame is not assembled
//  _q          :   frame generator object
//  _buffer     :   output buffer, [size: _buffer_len x 1]
//  _buffer_len :   output buffer length
int sc_flexframegen_write_samples(sc_flexframegen           _q,
                               liquid_float_complex * _buffer,
                               unsigned int           _buffer_len);


#endif // WIFLX_SCRATCH_SC_FLEXFRAMEGEN_H
