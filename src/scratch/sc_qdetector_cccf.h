#ifndef WIFLX_SCRATCH_SC_QDETECTOR_CCCF_H
#define WIFLX_SCRATCH_SC_QDETECTOR_CCCF_H

#include <liquid/liquid.h>

typedef struct sc_qdetector_cccf_s * sc_qdetector_cccf;

// create detector with generic sequence
//  _s      :   sample sequence
//  _s_len  :   length of sample sequence
sc_qdetector_cccf sc_qdetector_cccf_create(liquid_float_complex * _s,
                                     unsigned int           _s_len);

// create detector from sequence of symbols using internal linear interpolator
//  _sequence       :   symbol sequence
//  _sequence_len   :   length of symbol sequence
//  _ftype          :   filter prototype (e.g. LIQUID_FIRFILT_RRC)
//  _k              :   samples/symbol
//  _m              :   filter delay
//  _beta           :   excess bandwidth factor
sc_qdetector_cccf sc_qdetector_cccf_create_linear(liquid_float_complex * _sequence,
                                            unsigned int           _sequence_len,
                                            int                    _ftype,
                                            unsigned int           _k,
                                            unsigned int           _m,
                                            float                  _beta);

// create detector from sequence of GMSK symbols
//  _sequence       :   bit sequence
//  _sequence_len   :   length of bit sequence
//  _k              :   samples/symbol
//  _m              :   filter delay
//  _beta           :   excess bandwidth factor
sc_qdetector_cccf sc_qdetector_cccf_create_gmsk(unsigned char * _sequence,
                                          unsigned int    _sequence_len,
                                          unsigned int    _k,
                                          unsigned int    _m,
                                          float           _beta);

// create detector from sequence of CP-FSK symbols (assuming one bit/symbol)
//  _sequence       :   bit sequence
//  _sequence_len   :   length of bit sequence
//  _bps            :   bits per symbol, 0 < _bps <= 8
//  _h              :   modulation index, _h > 0
//  _k              :   samples/symbol
//  _m              :   filter delay
//  _beta           :   filter bandwidth parameter, _beta > 0
//  _type           :   filter type (e.g. LIQUID_CPFSK_SQUARE)
sc_qdetector_cccf sc_qdetector_cccf_create_cpfsk(unsigned char * _sequence,
                                           unsigned int    _sequence_len,
                                           unsigned int    _bps,
                                           float           _h,
                                           unsigned int    _k,
                                           unsigned int    _m,
                                           float           _beta,
                                           int             _type);

// Copy object including all internal objects and state
sc_qdetector_cccf sc_qdetector_cccf_copy(sc_qdetector_cccf _q);

int sc_qdetector_cccf_destroy(sc_qdetector_cccf _q);
int sc_qdetector_cccf_print  (sc_qdetector_cccf _q);
int sc_qdetector_cccf_reset  (sc_qdetector_cccf _q);

// run detector, looking for sequence; return pointer to aligned, buffered samples
void * sc_qdetector_cccf_execute(sc_qdetector_cccf       _q,
                              liquid_float_complex _x);

// get detection threshold
float sc_qdetector_cccf_get_threshold(sc_qdetector_cccf _q);

// set detection threshold (should be between 0 and 1, good starting point is 0.5)
int sc_qdetector_cccf_set_threshold(sc_qdetector_cccf _q,
                                 float          _threshold);

// set carrier offset search range
int sc_qdetector_cccf_set_range(sc_qdetector_cccf _q,
                             float          _dphi_max);

// access methods
unsigned int sc_qdetector_cccf_get_seq_len (sc_qdetector_cccf _q); // sequence length
const void * sc_qdetector_cccf_get_sequence(sc_qdetector_cccf _q); // pointer to sequence
unsigned int sc_qdetector_cccf_get_buf_len (sc_qdetector_cccf _q); // buffer length
float        sc_qdetector_cccf_get_rxy     (sc_qdetector_cccf _q); // correlator output
float        sc_qdetector_cccf_get_tau     (sc_qdetector_cccf _q); // fractional timing offset estimate
float        sc_qdetector_cccf_get_gamma   (sc_qdetector_cccf _q); // channel gain
float        sc_qdetector_cccf_get_dphi    (sc_qdetector_cccf _q); // carrier frequency offset estimate
float        sc_qdetector_cccf_get_phi     (sc_qdetector_cccf _q); // carrier phase offset estimate


#endif // WIFLX_SCRATCH_SC_QDETECTOR_CCCF_H
