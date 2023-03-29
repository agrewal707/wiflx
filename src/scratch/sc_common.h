#ifndef WIFLX_SCRATCH_SC_COMMON_H
#define WIFLX_SCRATCH_SC_COMMON_H

// report error
int sc_error_fl(int _code, const char * _file, int _line, const char * _format, ...);

// report error specifically for invalid object configuration 
void * sc_error_config_fl(const char * _file, int _line, const char * _format, ...);

// macro to get file name and line number for source of error
#define sc_error(code, format, ...) \
    sc_error_fl(code, __FILE__, __LINE__, format, ##__VA_ARGS__);

// macro to get file name and line number for source of error (invalid object)
#define sc_error_config(format, ...) \
    sc_error_config_fl(__FILE__, __LINE__, format, ##__VA_ARGS__);

#endif // WIFLX_SCRATCH_SC_COMMON_H
