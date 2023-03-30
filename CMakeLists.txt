#
# CMake configuration file for WIFLX
#
cmake_minimum_required (VERSION 2.8.12)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/modules/")
project (wiflx)

# project version
set (${PROJECT_NAME}_VERSION 0.0.1)

# options
option (WIFLX_RT_SCHED
  "Enable real rime thread scheduling" ON)
option (WIFLX_GPIO_DEBUG
  "Enable debugging TX/RX timing using GPIO" OFF)
option (WIFLX_TRACY_PROFILING
  "Profiling using Tracy profiler" OFF)
option (WIFLX_OFDM_DEBUG
  "Enable liquid-dsp OFDM debugging" OFF)
option (WIFLX_SC_DEBUG
    "Enable liquid-dsp Single Carrier (SC) debugging" OFF)
option (WIFLX_RADIO_DEBUG_SAMPLES
    "Enable radio samples debugging" OFF)
option (WIFLX_SIMULATION
  "Simulate WIFLX system" OFF)

# options header file
configure_file (
  "${PROJECT_SOURCE_DIR}/src/config.h.in"
  "${PROJECT_BINARY_DIR}/src/config.h"
)

set(CMAKE_CXX_FLAGS_FASTMATH "-Ofast" CACHE STRING
 "Flags used by the C++ compiler during fastmath builds."
  FORCE)

set(CMAKE_C_FLAGS_FASTMATH "-Ofast" CACHE STRING
  "Flags used by the C compiler during fastmath builds."
  FORCE)

set(CMAKE_EXE_LINKER_FLAGS_FASTMATH
  "-Wl,--warn-unresolved-symbols,--warn-once" CACHE STRING
  "Flags used for linking binaries during fastmath builds."
  FORCE)
set(CMAKE_SHARED_LINKER_FLAGS_FASTMATH
  "-Wl,--warn-unresolved-symbols,--warn-once" CACHE STRING
  "Flags used by the shared libraries linker during fastmath builds."
  FORCE)

if(${CMAKE_SYSTEM_PROCESSOR} MATCHES armv7l)
add_definitions (-mfpu=neon)
endif()

mark_as_advanced(
  CMAKE_CXX_FLAGS_FASTMATH
  CMAKE_C_FLAGS_FASTMATH
  CMAKE_EXE_LINKER_FLAGS_FASTMATH
  CMAKE_SHARED_LINKER_FLAGS_FASTMATH)

# Update the documentation string of CMAKE_BUILD_TYPE for GUIs
set(CMAKE_BUILD_TYPE "${CMAKE_BUILD_TYPE}" CACHE STRING
  "Choose the type of build, options are: None Debug Release RelWithDebInfo MinSizeRel Fastmath."
  FORCE)

add_subdirectory (src/common)
add_subdirectory (src/ap)
add_subdirectory (src/rm)
add_subdirectory (src/test)
add_subdirectory (src/scratch)
if(WIFLX_TRACY_PROFILING)
  add_subdirectory (profiling)
endif()
