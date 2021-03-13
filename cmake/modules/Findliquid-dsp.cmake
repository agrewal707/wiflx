INCLUDE(FindPkgConfig)
if(NOT LIQUID_DSP_FOUND)
  #pkg_check_modules(LIQUID_DSP_PKG liquid-dsp)

  find_path(LIQUID_DSP_INCLUDE_DIRS
    NAMES liquid/liquid.h
    NO_DEFAULT_PATH
    PATHS ${LIQUID_DSP_PKG_INCLUDE_DIRS}
          ${CMAKE_INSTALL_PREFIX}/include
  )

  find_library(LIQUID_DSP_LIBRARIES
    NAMES libliquid.so
    NO_DEFAULT_PATH
    PATHS ${LIQUID_DSP_PKG_LIBRARY_DIRS}
          ${CMAKE_INSTALL_PREFIX}/lib
  )

  find_library(LIQUID_DSP_STATIC_LIBRARIES
    NAMES libliquid.a
    NO_DEFAULT_PATH
    PATHS ${LIQUID_DSP_PKG_LIBRARY_DIRS}
          ${CMAKE_INSTALL_PREFIX}/lib
  )

if(LIQUID_DSP_INCLUDE_DIRS AND LIQUID_DSP_LIBRARIES)
  set(LIQUID_DSP_FOUND TRUE CACHE INTERNAL "liquid-dsp found")
  message(STATUS "Found liquid-dsp: ${LIQUID_DSP_INCLUDE_DIRS}, ${LIQUID_DSP_LIBRARIES}")
else()
  set(LIQUID_DSP_FOUND FALSE CACHE INTERNAL "liquid-dsp found")
  message(STATUS "liquid-dsp not found.")
endif()

mark_as_advanced(LIQUID_DSP_LIBRARIES LIQUID_DSP_INCLUDE_DIRS)

endif(NOT LIQUID_DSP_FOUND)
