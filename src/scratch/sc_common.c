#include "sc_common.h"

#include <stdarg.h>
#include <liquid/liquid.h>

// report error
int sc_error_fl(int          _code,
                    const char * _file,
                    int          _line,
                    const char * _format,
                    ...)
{
    va_list argptr;
    va_start(argptr, _format);
    fprintf(stderr,"error [%d]: %s\n", _code, liquid_error_info(_code));
    fprintf(stderr,"  %s:%u: ", _file, _line);
    vfprintf(stderr, _format, argptr);
    fprintf(stderr,"\n");
    va_end(argptr);
    return _code;
}

// report error
void * sc_error_config_fl(const char * _file,
                              int          _line,
                              const char * _format,
                              ...)
{
    int code = LIQUID_EICONFIG;
    va_list argptr;
    va_start(argptr, _format);
    fprintf(stderr,"error [%d]: %s\n", code, liquid_error_info(code));
    fprintf(stderr,"  %s:%u: ", _file, _line);
    vfprintf(stderr, _format, argptr);
    fprintf(stderr,"\n");
    va_end(argptr);
    return NULL;
}
