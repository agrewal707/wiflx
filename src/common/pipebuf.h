/*
 * Adapted from https://github.com/pabr/leansdr/blob/master/src/leansdr/framework.h
 * for use in WiFLX
 */
#ifndef WIFLX_COMMON_PIPEBUF_H
#define WIFLX_COMMON_PIPEBUF_H

#include <vector>
#include <complex>
#include <cstring>
#include <common/log.h>

namespace wiflx {
namespace common {

static const int MAX_READERS = 2;

struct pipebuf_common
{
  pipebuf_common(const char *_name) :
    name(_name)
  {}

  const char *name;
};

template<typename T>
struct pipebuf : pipebuf_common
{
  pipebuf (const char *name, unsigned long size)
    : pipebuf_common(name),
      buf (new T[size]),
      nrd(0),
      wr(buf),
      end(buf+size),
      min_write(1),
      total_written(0),
      total_read(0)
  {}

  int add_reader()
  {
    if (nrd == MAX_READERS )
    {
      WIFLX_LOG_ERROR("too many readers");
      return -1;
    }

    rds[nrd] = wr;
    return nrd++;
  }

  void pack()
  {
    T *rd = wr;
    for ( int i=0; i<nrd; ++i )
      if ( rds[i] < rd ) rd = rds[i];
    memmove(buf, rd, (wr-rd)*sizeof(T));
    wr -= rd - buf;
    for ( int i=0; i<nrd; ++i )
      rds[i] -= rd - buf;
  }

#ifdef DEBUG
  ~pipebuf()
  {
    WIFLX_LOG_FUNCTION (this << name);
  }
#endif

  T *buf;
  T *rds[MAX_READERS];
  int nrd;
  T *wr;
  T *end;
  unsigned long min_write;
  unsigned long total_written;
  unsigned long total_read;
};

template<typename T>
struct pipewriter
{
  pipewriter(pipebuf<T> &_buf, unsigned long min_write=1)
    : buf(_buf)
  {
    if ( min_write > buf.min_write )
      buf.min_write = min_write;
  }

  // Return number of items writable at this->wr, 0 if full.
  unsigned long writable()
  {
    if ( buf.end-buf.wr < buf.min_write )
      buf.pack();
    return buf.end - buf.wr;
  }

  T *wr() { return buf.wr; }

  void written(unsigned long n)
  {
    if ( buf.wr+n > buf.end )
    {
      WIFLX_LOG_ERROR("Bug: overflow to {}", buf.name);
      exit(1);
    }
    buf.wr += n;
    buf.total_written += n;
  }

  void write(const T &e)
  {
    *wr() = e;
    written(1);
  }

  pipebuf<T> &buf;
};

template<typename T>
struct pipereader
{
  pipereader(pipebuf<T> &_buf) :
    buf(_buf),
    id(_buf.add_reader())
  {}

  unsigned long readable() { return buf.wr - buf.rds[id]; }

  T *rd() { return buf.rds[id]; }

  void read(unsigned long n)
  {
    if ( buf.rds[id]+n > buf.wr )
    {
      WIFLX_LOG_ERROR("Bug: underflow from {}", buf.name);
      exit(1);
    }
    buf.rds[id] += n;
    buf.total_read += n;
  }

  pipebuf<T> &buf;
  int id;
};

using pipebuf_cf = pipebuf<std::complex<float>>;
using pipereader_cf = pipereader<std::complex<float>>;
using pipewriter_cf = pipewriter<std::complex<float>>;

} // namespace common
} // namespace wiflx

#endif // WIFLX_COMMON_PIPEBUF_H
