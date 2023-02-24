#include "cw.h"

#include <math.h>


void cw_write (float complex *buf_tx, int tx_buf_size, float A, float freq, float fs)
{
  float Ts = 1.0f / fs;
  for (int i = 0; i < tx_buf_size; ++i)
  {
    float phase = 2*M_PI*freq*i*Ts;
    buf_tx[i] = A*(cosf(phase) + _Complex_I * sinf(phase));
  }
}
