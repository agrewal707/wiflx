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
option (WIFLX_SIMULATION
  "Simulate WIFLX system" OFF)

# constants

# options header file
configure_file (
  "${PROJECT_SOURCE_DIR}/src/config.h.in"
  "${PROJECT_BINARY_DIR}/src/config.h"
)

add_subdirectory (src/common)
add_subdirectory (src/ap)
add_subdirectory (src/rm)
add_subdirectory (src/test)
