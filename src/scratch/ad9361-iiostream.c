// SPDX-License-Identifier: LGPL-2.1-or-later
/*
 * libiio - AD9361 IIO streaming example
 *
 * Copyright (C) 2014 IABG mbH
 * Author: Michael Feilen <feilen_at_iabg.de>
 **/
#define _GNU_SOURCE
#include <unistd.h>
#include <pthread.h>
#include <sched.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <signal.h>
#include <stdio.h>
#include <errno.h>
#include <iio.h>
#include <ad9361.h>
#include <complex.h>
#include <math.h>
#include <liquid/liquid.h>
#include "cw.h"
#include "sc_framegen.h"
#include "sc_framesync.h"
#include "sc_flexframegen.h"
#include "sc_flexframesync.h"

#define DEBUG_BUFFER_LEN 16384
 
/* helper macros */
#define MHZ(x) ((long long)(x*1000000.0 + .5))
#define GHZ(x) ((long long)(x*1000000000.0 + .5))
 
#define IIO_ENSURE(expr) { \
  if (!(expr)) { \
      (void) fprintf(stderr, "assertion failed (%s:%d)\n", __FILE__, __LINE__); \
      (void) abort(); \
  } \
}
 
/* RX is input, TX is output */
enum iodev { RX, TX };
 
/* common RX and TX streaming params */
struct stream_cfg {
  long long bw_hz; // Analog banwidth in Hz
  long long fs_hz; // Baseband sample rate in Hz
  long long lo_hz; // Local oscillator frequency in Hz
  const char* rfport; // Port name
};
 
/* static scratch mem for strings */
static char tmpstr[64];
 
/* IIO structs required for streaming */
static struct iio_context *ctx   = NULL;
static struct iio_device *tx = NULL;
static struct iio_device *rx = NULL;
static struct iio_channel *rx0_i = NULL;
static struct iio_channel *rx0_q = NULL;
static struct iio_channel *tx0_i = NULL;
static struct iio_channel *tx0_q = NULL;
static struct iio_buffer  *rxbuf = NULL;
static struct iio_buffer  *txbuf = NULL;
 
static bool stop;
 
/* cleanup and exit */
static void shutdown()
{
  printf("* Destroying buffers\n");
  if (rxbuf) { iio_buffer_destroy(rxbuf); }
  if (txbuf) { iio_buffer_destroy(txbuf); }

  printf("* Disabling streaming channels\n");
  if (rx0_i) { iio_channel_disable(rx0_i); }
  if (rx0_q) { iio_channel_disable(rx0_q); }
  if (tx0_i) { iio_channel_disable(tx0_i); }
  if (tx0_q) { iio_channel_disable(tx0_q); }

  printf("* Destroying context\n");
  if (ctx) { iio_context_destroy(ctx); }
  exit(0);
}
 
static void handle_sig(int sig)
{
  printf("Waiting for process to finish... Got signal %d\n", sig);
  stop = true;
}
 
/* check return value of attr_write function */
static void errchk(int v, const char* what) 
{
  if (v < 0) { fprintf(stderr, "Error %d writing to channel \"%s\"\nvalue may not be supported.\n", v, what); shutdown(); }
}
 
/* write attribute: long long int */
static void wr_ch_lli(struct iio_channel *chn, const char* what, long long val)
{
  errchk(iio_channel_attr_write_longlong(chn, what, val), what);
}
 
/* write attribute: string */
static void wr_ch_str(struct iio_channel *chn, const char* what, const char* str)
{
  errchk(iio_channel_attr_write(chn, what, str), what);
}
 
/* helper function generating channel names */
static char* get_ch_name(const char* type, int id)
{
  snprintf(tmpstr, sizeof(tmpstr), "%s%d", type, id);
  return tmpstr;
}
 
/* returns ad9361 phy device */
static struct iio_device* get_ad9361_phy(void)
{
  struct iio_device *dev = iio_context_find_device(ctx, "ad9361-phy");
  IIO_ENSURE(dev && "No ad9361-phy found");
  return dev;
}
 
/* finds AD9361 streaming IIO devices */
static bool get_ad9361_stream_dev(enum iodev d, struct iio_device **dev)
{
  switch (d) {
    case TX: *dev = iio_context_find_device(ctx, "cf-ad9361-dds-core-lpc"); return *dev != NULL;
    case RX: *dev = iio_context_find_device(ctx, "cf-ad9361-lpc");  return *dev != NULL;
    default: IIO_ENSURE(0); return false;
  }
}
 
/* finds AD9361 streaming IIO channels */
static bool get_ad9361_stream_ch(enum iodev d, struct iio_device *dev, int chid, struct iio_channel **chn)
{
  *chn = iio_device_find_channel(dev, get_ch_name("voltage", chid), d == TX);
  if (!*chn)
      *chn = iio_device_find_channel(dev, get_ch_name("altvoltage", chid), d == TX);
  return *chn != NULL;
}
 
/* finds AD9361 phy IIO configuration channel with id chid */
static bool get_phy_chan(enum iodev d, int chid, struct iio_channel **chn)
{
  switch (d) {
    case RX: *chn = iio_device_find_channel(get_ad9361_phy(), get_ch_name("voltage", chid), false); return *chn != NULL;
    case TX: *chn = iio_device_find_channel(get_ad9361_phy(), get_ch_name("voltage", chid), true);  return *chn != NULL;
    default: IIO_ENSURE(0); return false;
  }
}
 
/* finds AD9361 local oscillator IIO configuration channels */
static bool get_lo_chan(enum iodev d, struct iio_channel **chn)
{
  switch (d) {
    // LO chan is always output, i.e. true
    case RX: *chn = iio_device_find_channel(get_ad9361_phy(), get_ch_name("altvoltage", 0), true); return *chn != NULL;
    case TX: *chn = iio_device_find_channel(get_ad9361_phy(), get_ch_name("altvoltage", 1), true); return *chn != NULL;
    default: IIO_ENSURE(0); return false;
  }
}
 
/* applies streaming configuration through IIO */
bool cfg_ad9361_streaming_ch(struct stream_cfg *cfg, enum iodev type, int chid)
{
  struct iio_channel *chn = NULL;

  // Configure phy and lo channels
  printf("* Acquiring AD9361 phy channel %d for %s\n", chid, type == TX ? "TX" : "RX");
  if (!get_phy_chan(type, chid, &chn)) {  return false; }
  wr_ch_str(chn, "rf_port_select",     cfg->rfport);
  wr_ch_lli(chn, "rf_bandwidth",       cfg->bw_hz);

  long long fs_hz = cfg->fs_hz;
  if (fs_hz < (25e6 / (12*4)))
    // It is assumed that decimation/interpolation will be activated on FPGA
    fs_hz = cfg->fs_hz*8;

  printf("* Set sampling frequency %lli on AD9361 phy channel %d for type %s\n", fs_hz, chid, type == TX ? "TX" : "RX");
  wr_ch_lli(chn, "sampling_frequency", fs_hz);

  // Configure LO channel
  printf("* Acquiring AD9361 %s lo channel\n", type == TX ? "TX" : "RX");
  if (!get_lo_chan(type, &chn)) { return false; }
  wr_ch_lli(chn, "frequency", cfg->lo_hz);
  return true;
}

#define FIR_BUF_SIZE  8192
int load_fir_filter (struct iio_device *dev, const char *filename)
{
  int ret = 0;

  char *buf = malloc(FIR_BUF_SIZE);
  if (!buf) {
    goto done;
  }

  FILE *file = fopen (filename, "r");
  if (!file) {
    fprintf(stderr, "Can't open %s, err: %s\n", filename, strerror(errno));
    goto done;
  }

  char line[256]; int len = 0, lines = 0;
  while (fgets (line, sizeof(line), file)) 
  {
    if (line[0] == '#')
      continue;

    printf ("line: %s", line);
    len += snprintf (buf + len, FIR_BUF_SIZE - len, "%s",line);
    lines++;
  }
  len += snprintf(buf + len, FIR_BUF_SIZE - len, "\n");
  // 6 lines for headers
  printf ("taps: %d, len: %d\n", lines - 6, len);

  ret = iio_device_attr_write_raw(dev, "filter_fir_config", buf, len);
  if (ret < 0)
    ret = 0;

  printf ("RET :%d\n", ret);

done:
  if (buf)
    free (buf);

  if (file)
    fclose (file);

  return ret;
}

windowcf debug_tx;
windowcf debug_rx;
void dump_debug_data (const char *_filename, float fs)
{
  float complex *rc;
  int i;
  FILE* fid = fopen(_filename,"w");
  if (!fid) {
      printf("could not open %s for writing\n", _filename);
      return;
  }

  fprintf(fid,"%% %s: auto-generated file", _filename);
  fprintf(fid,"\n\n");
  fprintf(fid,"clear all;\n");
  fprintf(fid,"close all;\n\n");
  fprintf(fid,"n = %u;\n", DEBUG_BUFFER_LEN);

  // write TX
  fprintf(fid,"tx = zeros(1,n);\n");
  windowcf_read(debug_tx, &rc);
  for (i=0; i<DEBUG_BUFFER_LEN; i++)
  fprintf(fid,"tx(%4u) = %12.4e + j*%12.4e;\n", i+1, creal(rc[i]), cimag(rc[i]));
  fprintf(fid,"\n\n");

  // TX figure
  fprintf(fid,"figure('Name', 'TX', 'Color','white','position',[100 100 800 600]);\n");
  fprintf(fid,"plotspec2(tx(1:end), %f, [0 60], [-100 0]);\n", 1/fs);

  // write RX
  fprintf(fid,"rx = zeros(1,n);\n");
  windowcf_read(debug_rx, &rc);
  for (i=0; i<DEBUG_BUFFER_LEN; i++)
  fprintf(fid,"rx(%4u) = %12.4e + j*%12.4e;\n", i+1, creal(rc[i]), cimag(rc[i]));
  fprintf(fid,"\n\n");

  // RX figure
  fprintf(fid,"figure('Name', 'RX', 'Color','white','position',[100 100 800 600]);\n");
  fprintf(fid,"plotspec2(rx(1:end), %f, [0 6], [-100 0]);\n", 1/fs);

  fprintf(fid,"\n\n");
  fclose(fid);

  printf ("tx/rx samples written to %s\n", _filename);
}

void dump_tx_frame (const char *_filename, float complex *x, int x_len, float fs)
{
  float complex *rc;
  int i;
  FILE* fid = fopen(_filename,"w");
  if (!fid) {
      printf("could not open %s for writing: %s\n", _filename, strerror(errno));
      return;
  }

  fprintf(fid,"%% %s: auto-generated file", _filename);
  fprintf(fid,"\n\n");
  fprintf(fid,"clear all;\n");
  fprintf(fid,"close all;\n\n");
  fprintf(fid,"n = %u;\n", x_len);

  // write TX
  fprintf(fid,"tx = zeros(1,n);\n");
  for (i=0; i<x_len; i++)
  fprintf(fid,"tx(%4u) = %12.4e + j*%12.4e;\n", i+1, creal(x[i]), cimag(x[i]));
  fprintf(fid,"\n\n");
  fprintf(fid,"plotspec2(tx, %.8f, [ 0 0 ], [ -100 0 ]);\n", 1/fs);

  fclose(fid);

  printf ("tx samples written to %s\n", _filename);
}

/* simple configuration and streaming */
/* usage:
 * Default context, assuming local IIO devices, i.e., this script is run on ADALM-Pluto for example
 $./a.out
 * URI context, find out the uri by typing `iio_info -s` at the command line of the host PC
 $./a.out usb:x.x.x
 */

struct config
{
  char *uri;
  float rxfreq;
  float txfreq;
  float rxgain;
  float txgain;
  char *filter;
  float rxbw;
  float txbw;
  float fs;
  bool rxen;
  bool txen;
  bool cw;
  bool sc64;
  bool scff;
  unsigned int scff_ms;
  unsigned int scff_fec0;
  unsigned int scff_fec1;
  unsigned int scff_check;
  unsigned int scff_dlen;
  bool noise;
  float noise_floor;
  unsigned int M;
  char *debug_prefix;
  bool lo_offset_en;
  float lo_offset;
  char *tx_waveform_file;
};

void set_thread_param (pthread_t th, const char *name, int policy, int priority, int cpu)
{
  pthread_setname_np (th, name);

  if (-1 != cpu)
  {
    cpu_set_t cpu_set;
    CPU_ZERO(&cpu_set);
    CPU_SET(cpu, &cpu_set);
    if (pthread_setaffinity_np (th, sizeof(cpu_set_t), &cpu_set))
      fprintf(stderr, "failed to set cpu affinity on %s\n", name);
  }

  if (-1 != policy)
  {
    struct sched_param p;
    p.sched_priority = priority;
    if (pthread_setschedparam(th, policy, &p))
      fprintf(stderr, "failed to set policy/priority on %s\n", name);
  }

  {
    cpu_set_t cpu_set;
    CPU_ZERO(&cpu_set);
    pthread_getaffinity_np(th, sizeof(cpu_set_t), &cpu_set);
    fprintf(stdout, "%s thread CPU mask: ", name);
    for (int i=0; i < 4; ++i)
      fprintf(stdout, "%d\n",CPU_ISSET(i, &cpu_set));
  }

  {
    struct sched_param p;
    if (!pthread_getschedparam(th, &policy, &p))
      fprintf(stderr, "%s thread, policy %d, priority %d", name, policy, p.sched_priority);
  }
}

void usage (const char *name, FILE *f, int ret, const char *info)
{
  fprintf(f, "Usage: %s [options]\n", name);
  fprintf
    (f,
     "\nOptions:\n"
     "  --uri STRING         Device URI\n"
     "  --rxfreq FLOAT       Recieve frequency\n"
     "  --txfreq FLOAT       Recieve frequency\n"
     "  --rxgain FLOAT       RX gain (dBm)\n"
     "  --txgain FLOAT       TX gain (dBm)\n"
     "  --filter STRING      FIR filter\n"
     "  --rxbw FLOAT         RF RX bandwidth (Hz)\n"
     "  --txbw FLOAT         RF TX bandwidth (Hz)\n"
     "  --fs FLOAT           Sampling rate (Hz)\n"
     "  --M UNSIGNED         Decimation factor\n"
     "  -d                   show debug logs\n"
     );
  if (info)
    fprintf(f, "Error while processing '%s'\n", info);
  exit(ret);
}

// static callback function
static int rx_callback(unsigned char *  _header,
                    int              _header_valid,
                    unsigned char *  _payload,
                    unsigned int     _payload_len,
                    int              _payload_valid,
                    framesyncstats_s _stats,
                    void *           _userdata)
{
    static int num_debug_files = 7;
    printf("*** callback invoked ***: HDR:%u, PAYLOAD:%u\n", _header_valid, _payload_valid);
    framesyncstats_print(&_stats);
    if (--num_debug_files < 0)
      return 0;

    return -2;
}

#define min(a,b) \
 ({ __typeof__ (a) _a = (a); \
     __typeof__ (b) _b = (b); \
   _a < _b ? _a : _b; })

void* plutosdr_monitor(void *data)
{
  uint32_t rxval, txval;
  int ret;

  struct config *cfg = (struct config*)data;

  if (! (tx || rx)) {
    fprintf (stderr, "TX or RX streaming devices not intilized\n");
    return NULL;
  }

  // Clear all status bits for TX and RX dev
  iio_device_reg_write(tx, 0x80000088, 0x6);
  iio_device_reg_write(rx, 0x80000088, 0x6);

  while (!stop) {
    if (cfg->txen) {
      // Check TX device
      ret = iio_device_reg_read(tx, 0x80000088, &txval);
      if (ret) {
        fprintf(stderr, "Monitor: Failed to read status register: %s\n",
            strerror(-ret));
      } else if (txval & 1)
        fprintf(stderr, "Monitor: TX DEVICE UNDERFLOW DETECTED!\n");

      // Clear bits
      if (txval)
        iio_device_reg_write(tx, 0x80000088, txval);
    }

    if (cfg->rxen) {
      // Check RX device
      ret = iio_device_reg_read(rx, 0x80000088, &rxval);
      if (ret) {
        fprintf(stderr, "Monitor: Failed to read status register: %s\n",
            strerror(-ret));
      } else if (rxval & 4)
        fprintf (stderr, "Monitor: RX DEVICE OVERFLOW DETECTED!\n");

      // Clear bits
      if (rxval)
        iio_device_reg_write(rx, 0x80000088, rxval);
    }

    sleep(1);
  }

  return NULL;
}

int main (int C, char **V)
{
  // RX and TX sample counters
  size_t nrx = 0;
  size_t ntx = 0;

  // Stream configurations
  struct stream_cfg rxcfg;
  struct stream_cfg txcfg;

  struct config cfg;
  cfg.rxfreq = MHZ(787.5);
  cfg.txfreq = MHZ(757.5);
  cfg.rxgain = 50.0;
  cfg.txgain = -29.0;
  cfg.filter = "../pluto_1024ksps.ftr";
  cfg.rxbw = 1026412;
  cfg.txbw = 1257350;
  cfg.fs = 1024000;
  cfg.rxen = false;
  cfg.txen = false;
  cfg.cw = false;
  cfg.sc64 = false;
  cfg.scff = false;
  cfg.scff_ms = liquid_getopt_str2mod("qpsk");
  cfg.scff_fec0 = liquid_getopt_str2fec("none");
  cfg.scff_fec1 = liquid_getopt_str2fec("none");
  cfg.scff_check = liquid_getopt_str2crc("crc32");
  cfg.scff_dlen = 150;
  cfg.noise = false;
  cfg.noise_floor = -60.0f;
  cfg.M = 1;
  cfg.debug_prefix = "sc_framesync";
  cfg.lo_offset_en = false;
  cfg.lo_offset = 50e3; // Khz
  cfg.tx_waveform_file = NULL;

  // Listen to ctrl+c and IIO_ENSURE
  signal(SIGINT, handle_sig);


  if (C < 2)
  {
    usage(V[0], stderr, 1, 0);
  }

  for (int i = 1; i < C; ++i)
  {
    if (!strcmp(V[i], "--uri") && i+1<C)
      cfg.uri = V[++i];
    else if (!strcmp(V[i], "--rxfreq") && i+1<C)
      cfg.rxfreq = atof(V[++i]);
    else if (!strcmp(V[i], "--rxgain") && i+1<C)
      cfg.rxgain = atof(V[++i]);
    else if (!strcmp(V[i], "--txfreq") && i+1<C)
      cfg.txfreq = atof(V[++i]);
    else if (!strcmp(V[i], "--txgain") && i+1<C)
      cfg.txgain = atof(V[++i]);
    else if (!strcmp(V[i], "--filter") && i+1<C)
      cfg.filter = V[++i];
    else if (!strcmp(V[i], "--rxbw") && i+1<C)
      cfg.rxbw = atof(V[++i]);
    else if (!strcmp(V[i], "--txbw") && i+1<C)
      cfg.txbw = atof(V[++i]);
    else if (!strcmp(V[i], "--fs") && i+1<C)
      cfg.fs = atof(V[++i]);
    else if (!strcmp(V[i], "-cw"))
      cfg.cw = true;
    else if (!strcmp(V[i], "-sc64"))
      cfg.sc64 = true;
    else if (!strcmp(V[i], "-scff"))
      cfg.scff = true;
    else if (!strcmp(V[i], "--scff_ms") && i+1 < C)
      cfg.scff_ms = liquid_getopt_str2mod(V[++i]);
    else if (!strcmp(V[i], "--scff_fec0") && i+1 < C)
      cfg.scff_fec0 = liquid_getopt_str2fec(V[++i]);
    else if (!strcmp(V[i], "--scff_fec1") && i+1 < C)
      cfg.scff_fec1 = liquid_getopt_str2fec(V[++i]);
    else if (!strcmp(V[i], "--scff_check") && i+1 < C)
      cfg.scff_check = liquid_getopt_str2crc(V[++i]);
    else if (!strcmp(V[i], "--scff_dlen") && i+1 < C)
      cfg.scff_dlen = atoi(V[++i]);
    else if (!strcmp(V[i], "--tx-waveform-file") && i+1 < C)
      cfg.tx_waveform_file = V[++i];
    else if (!strcmp(V[i], "--noise") && i+1 < C) {
      cfg.noise = true;
      cfg.noise_floor = atof(V[++i]);
   }else if (!strcmp(V[i], "--M") && i+1 < C) {
      cfg.M = atoi(V[++i]);
   }else if (!strcmp(V[i], "--lo-offset-en")) {
      cfg.lo_offset_en = true;
   }else if (!strcmp(V[i], "--lo-offset") && i+1 < C) {
      cfg.lo_offset = atof(V[++i]);
   }else if (!strcmp(V[i], "--debug-prefix") && i+1 < C) {
      cfg.debug_prefix = V[++i];
   }else if (!strcmp(V[i], "-rxen"))
      cfg.rxen = true;
    else if (!strcmp(V[i], "-txen"))
      cfg.txen = true;
    else if (!strcmp(V[i], "-h"))
      usage (V[0], stdout, 0, 0);
    else
      usage(V[0], stderr, 1, V[i]);
  }

  // RX stream config
  //rxcfg.bw_hz = 1026412;
  //rxcfg.fs_hz = 1024000;   // 1024 KS/s rx sample rate
  //rxcfg.lo_hz = MHZ(787.5); // 787.5 MHz rf frequency
  //rxcfg.rfport = "A_BALANCED"; // port A (select for rf freq.)

  // TX stream config
  //txcfg.bw_hz = 1257350;
  //txcfg.fs_hz = 1024000;   // 1024 KS/s tx sample rate
  //txcfg.lo_hz = MHZ(757.5); // 757.5 MHz rf frequency
  //txcfg.rfport = "A"; // port A (select for rf freq.)

  // RX stream config
  rxcfg.bw_hz = cfg.rxbw;
  rxcfg.fs_hz = cfg.fs;
  rxcfg.lo_hz = cfg.rxfreq;
  rxcfg.rfport = "A_BALANCED"; // port A (select for rf freq.)

  // TX stream config
  txcfg.bw_hz = cfg.txbw;
  txcfg.fs_hz = cfg.fs;
  txcfg.lo_hz = cfg.txfreq; // 757.5 MHz rf frequency
  txcfg.rfport = "A"; // port A (select for rf freq.)

  printf("* Acquiring IIO context\n");
  //IIO_ENSURE((ctx = iio_create_default_context()) && "No context");
  IIO_ENSURE((ctx = iio_create_context_from_uri(cfg.uri)) && "No context");
  IIO_ENSURE(iio_context_get_devices_count(ctx) > 0 && "No devices");

  printf("* Acquiring AD9361 streaming devices\n");
  IIO_ENSURE(get_ad9361_stream_dev(TX, &tx) && "No tx dev found");
  IIO_ENSURE(get_ad9361_stream_dev(RX, &rx) && "No rx dev found");

  printf("* Enabling FIR filter on phy channels\n");
  IIO_ENSURE(load_fir_filter(get_ad9361_phy(), cfg.filter));

  printf("* Enabling FIR filter on phy channels\n");
  struct iio_channel *chn = NULL;
  IIO_ENSURE(get_phy_chan(RX, 0, &chn) && "RX port 0 not found");
  wr_ch_lli(chn, "filter_fir_en", 1);
  IIO_ENSURE(get_phy_chan(TX, 0, &chn) && "TX port 0 not found");
  wr_ch_lli(chn, "filter_fir_en", 1);

  // set buffer count
  printf("* Configure kernel buffer count for TXRX\n");
  if (iio_device_set_kernel_buffers_count(tx, 4) != 0) {
    printf("Error configuring kernel buffer count for TX!\n");
  }
  if (iio_device_set_kernel_buffers_count(rx, 6) != 0) {
    printf("Error configuring kernel buffer count for RX!\n");
  }

  printf("* Configuring AD9361 for streaming\n");
  IIO_ENSURE(cfg_ad9361_streaming_ch(&rxcfg, RX, 0) && "RX port 0 not found");
  IIO_ENSURE(cfg_ad9361_streaming_ch(&txcfg, TX, 0) && "TX port 0 not found");

  printf("* Initializing AD9361 IIO streaming channels\n");
  IIO_ENSURE(get_ad9361_stream_ch(RX, rx, 0, &rx0_i) && "RX chan i not found");
  IIO_ENSURE(get_ad9361_stream_ch(RX, rx, 1, &rx0_q) && "RX chan q not found");
  IIO_ENSURE(get_ad9361_stream_ch(TX, tx, 0, &tx0_i) && "TX chan i not found");
  IIO_ENSURE(get_ad9361_stream_ch(TX, tx, 1, &tx0_q) && "TX chan q not found");

  // Enable dec/int stage of cf-ad9361-lpc / cf-ad9361-dds-core-lpc
  wr_ch_lli(rx0_i, "sampling_frequency", rxcfg.fs_hz);
  wr_ch_lli(tx0_i, "sampling_frequency", txcfg.fs_hz);

  printf("* Set TX gain\n");
  // Set TX gain
  get_phy_chan(TX, 0, &chn);
  wr_ch_lli(chn, "hardwaregain", cfg.txgain);

  // Set RX gain
  printf("* Set RX gain\n");
  get_phy_chan(RX, 0, &chn);
  wr_ch_str(chn, "gain_control_mode", "manual"); // fast_attack, slow_attack, manual
  wr_ch_lli(chn, "hardwaregain", cfg.rxgain);

  printf("* Enabling IIO streaming channels\n");
  iio_channel_enable(rx0_i);
  iio_channel_enable(rx0_q);
  iio_channel_enable(tx0_i);
  iio_channel_enable(tx0_q);

  const int iio_rx_buf_size = WIFLX_SC_LIQUID_FRAME64_LEN;
  //const int iio_rx_buf_size = 256;
  printf("* Creating non-cyclic RX IIO buffers with %u samples\n", iio_rx_buf_size);
  rxbuf = iio_device_create_buffer(rx, iio_rx_buf_size, false);
  if (!rxbuf) {
    perror("Could not create RX buffer");
    shutdown();
  }
  const int iio_tx_buf_size = WIFLX_SC_LIQUID_FRAME64_LEN;
  //const int iio_tx_buf_size = 256;
  printf("* Creating non-cyclic TX IIO buffers with %u samples\n", iio_tx_buf_size);
  txbuf = iio_device_create_buffer(tx, iio_tx_buf_size, false);
  if (!txbuf) {
    perror("Could not create TX buffer");
    shutdown();
  }

  // Turn ON RX LO
  get_lo_chan(RX, &chn);
	iio_channel_attr_write_bool(chn, "powerdown", false);
  // Turn ON TX LO
  get_lo_chan(TX, &chn);
	iio_channel_attr_write_bool(chn, "powerdown", false);

  int tx_buf_size = 32768;
  float complex buf_tx[tx_buf_size];

  sc_framegen fg = NULL;
  sc_framesync fs = NULL;

  sc_flexframegen ffg = NULL;
  sc_flexframesync ffs = NULL;

  uint32_t pkt_id = 0;

  unsigned char hdr[8];
  *((uint32_t*)hdr) = 0;
  for (int k = 4; k < 8; ++k)
    hdr[k] = 0xAA;
  unsigned char data[1500];
  for (int k = 0; k < 1500; ++k)
    data[k] = 0xAA;


  nco_crcf cw_mixer = NULL;
  if (cfg.cw) {
#if 1
    cw_mixer = nco_crcf_create(LIQUID_NCO);
    nco_crcf_set_phase(cw_mixer, 0.0f);
    nco_crcf_set_frequency(cw_mixer, 2.0f*M_PI*16e3/cfg.fs);
    for (int i = 0; i < iio_tx_buf_size; ++i) {
      float complex y;
      nco_crcf_cexpf(cw_mixer, &buf_tx[i]);
      nco_crcf_step(cw_mixer);
    }
    //nco_crcf_destroy(cw_mixer);
#else
    cw_write(buf_tx, iio_tx_buf_size, 1.0f, 16e3, txcfg.fs_hz);
#endif
  } else if (cfg.sc64) {
    if (cfg.txen) {
      fg = sc_framegen_create();
      sc_framegen_execute (fg, hdr, data, buf_tx, &tx_buf_size);
      sc_framegen_print (fg);
      if (cfg.tx_waveform_file)
        dump_tx_frame (cfg.tx_waveform_file, buf_tx, tx_buf_size, cfg.fs);
    }

    if (cfg.rxen) {
     fs = sc_framesync_create (rx_callback, NULL);
     sc_framesync_set_prefix(fs, cfg.debug_prefix);
     sc_framesync_print (fs);
    }
  } else if (cfg.scff) {
    if (cfg.txen) {
      sc_flexframegenprops_s props = {
        .check = cfg.scff_check,
        .fec0 = cfg.scff_fec0,
        .fec1 = cfg.scff_fec1,
        .mod_scheme = cfg.scff_ms
      };
      ffg = sc_flexframegen_create(&props);
      sc_flexframegen_assemble (ffg, hdr, data, cfg.scff_dlen);
      const int frame_len = sc_flexframegen_getframelen(ffg);
      if (frame_len > tx_buf_size) {
        fprintf (stderr, "Frame length %u exceeds tx buf size %u\n", frame_len, tx_buf_size);
        shutdown();
      }
      const bool frame_complete = sc_flexframegen_write_samples (ffg, buf_tx, frame_len);
      if (!frame_complete) {
        fprintf (stderr, "Frame NOT complete\n");
        shutdown();
      }
      tx_buf_size = frame_len;
      printf ("Wrote %u samples to tx buffer\n", tx_buf_size);

      sc_flexframegen_print (ffg);
      if (cfg.tx_waveform_file)
        dump_tx_frame (cfg.tx_waveform_file, buf_tx, tx_buf_size, cfg.fs);
    }

    if (cfg.rxen) {
     ffs = sc_flexframesync_create (rx_callback, NULL);
     //sc_framesync_set_prefix(fs, cfg.debug_prefix);
     sc_flexframesync_print (ffs);

    }
  }

  float max_sample_val = 0.0f;
  for (int i = 0; i < tx_buf_size; ++i) {
    float _i = crealf(buf_tx[i]);
    float _q = cimagf(buf_tx[i]);
    //printf ("TX sample(%u) = %12.4e + 1i*%12.4e, %f, %f\n", i, _i, _q, fabsf(_i), fabsf(_q));
    if (fabsf(_i) > max_sample_val)
      max_sample_val = fabsf(_i);
    if (fabsf(_q) > max_sample_val)
      max_sample_val = fabsf(_q);
  }
  if (max_sample_val < 1.0f)
    max_sample_val = 1.0f;
  printf ("Maximum sample val = %.6f\n", max_sample_val);

  // 12-bit sample needs to be MSB aligned so shift by 4
  // https://wiki.analog.com/resources/eval/user-guides/ad-fmcomms2-ebz/software/basic_iq_datafiles#binary_format
  // AD9361 12-bit MSB aligned [(2^(12-1) - 1) * 16 =  32752]
  float tx_scale = floor(((powf(2, 12-1)-1)*16) / max_sample_val);
  printf ("TX scale = %.6f\n", tx_scale);
  //tx_scale = 8192;
  printf ("TX scale (forced) = %.6f\n", tx_scale);

  if (cfg.noise) {
    float nstd  = powf(10.0f, cfg.noise_floor/20.0f); // noise std. dev.
    for (int i = 0; i < tx_buf_size; ++i)
      buf_tx[i] += nstd*(randnf() + _Complex_I*randnf())*M_SQRT1_2;
  }

  //set_thread_param (pthread_self(), "PHY", SCHED_FIFO, 1, 0);
 
  debug_tx = windowcf_create (DEBUG_BUFFER_LEN);
  debug_rx = windowcf_create (DEBUG_BUFFER_LEN);

  firdecim_crcf decim;
  msresamp2_crcf mdecim;
  if (cfg.M > 1) {
#if 0
    // For fs = 320Khz, fc = 40Khz = 0.125*fs, df (transition bandwidth) = (30-50)/fs = 0.0625
    const int h_len = estimate_req_filter_len(0.0625, 80.0f);
    //const int h_len = 30;
    printf("h_len: %d\n", h_len);
    float h[h_len];
    // fc = 320 * 0.25 = 80
    liquid_firdes_kaiser(h_len, 0.125, 80.0f, 0.0f, h);
    decim = firdecim_crcf_create(cfg.M, h, h_len);
    firdecim_crcf_set_scale(decim, 1.0f/(float)cfg.M);
#endif

#if 1
    decim = firdecim_crcf_create_kaiser(cfg.M, 7, 80.0f);
    firdecim_crcf_set_scale(decim, 1.0f/(float)cfg.M);
#endif

#if 0
  // For fs = 320Khz, fc = 40Khz = 0.125*fs
  mdecim = msresamp2_crcf_create(LIQUID_RESAMP_DECIM, cfg.M, 1.0f/(cfg.M*4), 0.0f, 80.0f);
#endif
  }

  nco_crcf if_mixer = nco_crcf_create(LIQUID_NCO);
  nco_crcf_set_phase(if_mixer, 0.0f);
  nco_crcf_set_frequency(if_mixer, 2.0f*M_PI*cfg.lo_offset/cfg.fs);

  pthread_t monitor;
  pthread_create (&monitor, NULL, plutosdr_monitor, &cfg);

  size_t rx_i = 0;
  printf("* Starting IO streaming (press CTRL+C to cancel)\n");
  while (!stop)
  {
    ssize_t nbytes_rx, nbytes_tx;

    if (cfg.rxen) { // RX - READ
      // Refill RX buffer
      nbytes_rx = iio_buffer_refill(rxbuf);
      if (nbytes_rx < 0) { printf("Error refilling buf %d\n",(int) nbytes_rx); shutdown(); }

      // READ: Get pointers to RX buf and read IQ from RX buf port 0
      ptrdiff_t p_inc = iio_buffer_step(rxbuf);
      char *p_end = iio_buffer_end(rxbuf);
      float complex x_if; // sample at IF frequency
      float complex x[cfg.M];
      float complex y;
      for (char *p_dat = (char *)iio_buffer_first(rxbuf, rx0_i); p_dat < p_end; p_dat += p_inc) {
        const int16_t _i = ((int16_t*)p_dat)[0]; // Real (I)
        const int16_t _q = ((int16_t*)p_dat)[1]; // Imag (Q)

        x_if = _i / 2048.0f + I * _q / 2048.0f;

        if (cfg.lo_offset_en) {
          nco_crcf_mix_down(if_mixer, x_if, &x_if);
          nco_crcf_step(if_mixer);
        }

        x[rx_i++] = x_if;
        if (cfg.M > 1) {
          if (rx_i == cfg.M) {
            firdecim_crcf_execute(decim, x, &y);
            //msresamp2_crcf_execute (mdecim, x, &y);
            if (cfg.sc64)
              sc_framesync_execute (fs, &y, 1);
            else
              sc_flexframesync_execute (ffs, &y, 1);

            windowcf_write (debug_rx, x, cfg.M);
            rx_i = 0;
          }
        } else {
          if (cfg.sc64)
            sc_framesync_execute (fs, x, 1);
          else
            sc_flexframesync_execute (ffs, x, 1);

          windowcf_write (debug_rx, x, 1);
          rx_i = 0;
        }
      }
      //printf("rx_i: %lu, nbytes_rx: %ld, samples: %ld\n", rx_i, nbytes_rx, nbytes_rx/iio_device_get_sample_size(rx));
    } // rx_en

    if (cfg.txen) { // TX WRITE: Get pointers to TX buf and write IQ to TX buf port 0
      if (cfg.sc64) {
        *((uint32_t*)hdr) = pkt_id++;
        sc_framegen_execute (fg, hdr, data, buf_tx, &tx_buf_size);
      } else if (cfg.scff) {
        *((uint32_t*)hdr) = pkt_id++;
        sc_flexframegen_assemble (ffg, hdr, data, cfg.scff_dlen);
        const int frame_len = sc_flexframegen_getframelen(ffg);
        const bool frame_complete = sc_flexframegen_write_samples (ffg, buf_tx, frame_len);
        tx_buf_size = frame_len;
      }

#if 0
else if (cfg.cw) {
        p_inc = iio_buffer_step(txbuf);
        p_end = iio_buffer_end(txbuf);
        for (p_dat = (char *)iio_buffer_first(txbuf, tx0_i); p_dat < p_end; p_dat += p_inc) {
          ((int16_t*)p_dat)[0] = (int16_t)(nco_crcf_cos(cw_mixer) * tx_scale); // Real (I)
          ((int16_t*)p_dat)[1] = (int16_t)(nco_crcf_sin(cw_mixer) * tx_scale); // Image (Q)
          nco_crcf_step (cw_mixer);
        }
      }
#endif

      for (size_t tx_i = 0; tx_i < tx_buf_size;) {
        char *p_dat = (char *)iio_buffer_first(txbuf, tx0_i);
        ptrdiff_t p_inc = iio_buffer_step(txbuf);
        char *p_end = iio_buffer_end(txbuf);

        const size_t samples_to_write = min(tx_buf_size - tx_i, iio_tx_buf_size);
        //printf ("samples to write: %lu\n", samples_to_write);

        for (int buf_i = 0; buf_i < samples_to_write; ++buf_i, p_dat += p_inc) {
          ((int16_t*)p_dat)[0] = (int16_t)(crealf(buf_tx[buf_i]) * tx_scale); // Real (I)
          ((int16_t*)p_dat)[1] = (int16_t)(cimagf(buf_tx[buf_i]) * tx_scale); // Image (Q)
        }
        // pad iio buffer (if needed)
        if (samples_to_write < iio_tx_buf_size)
          memset(p_dat, 0, p_end-p_dat);
         
        // Push iio buffer to the hardware
        nbytes_tx = iio_buffer_push(txbuf);
        //printf("tx_i: %lu, nbytes_tx: %ld, samples: %ld\n", tx_i, nbytes_tx, nbytes_tx/iio_device_get_sample_size(tx));
        if (nbytes_tx < 0) { printf("Error pushing buf %d\n", (int) nbytes_tx); shutdown(); }
        windowcf_write (debug_tx, buf_tx + tx_i, samples_to_write);
        tx_i += samples_to_write;
      }
    } // tx_en

    // Sample counter increment and status output
    nrx += nbytes_rx / iio_device_get_sample_size(rx);
    ntx += nbytes_tx / iio_device_get_sample_size(tx);
    //printf("\tRX %8.2f MSmp, TX %8.2f MSmp\n", nrx/1e6, ntx/1e6);
  }

  dump_debug_data ("/tmp/pluto_radio_samples.m", txcfg.fs_hz);
 
  shutdown();

  pthread_join (monitor, NULL);

  return 0;
}
