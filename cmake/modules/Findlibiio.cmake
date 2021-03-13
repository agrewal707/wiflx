INCLUDE(FindPkgConfig)
if(NOT LIBIIO_FOUND)
  #pkg_check_modules(LIBIIO_PKG libiio)

  find_path(LIBIIO_INCLUDE_DIRS
    NAMES iio.h
    NO_DEFAULT_PATH
    PATHS ${LIBIIO_PKG_INCLUDE_DIRS}
          ${CMAKE_INSTALL_PREFIX}/include
  )

  find_library(LIBIIO_LIBRARIES
    NAMES libiio.so
    NO_DEFAULT_PATH
    PATHS ${LIBIIO_PKG_LIBRARY_DIRS}
          ${CMAKE_INSTALL_PREFIX}/lib
  )

if(LIBIIO_INCLUDE_DIRS AND LIBIIO_LIBRARIES)
  set(LIBIIO_FOUND TRUE CACHE INTERNAL "libiio found")
  message(STATUS "Found libiio: ${LIBIIO_INCLUDE_DIRS}, ${LIBIIO_LIBRARIES}")
else()
  set(LIBIIO_FOUND FALSE CACHE INTERNAL "libiio found")
  message(STATUS "libiio not found.")
endif()

mark_as_advanced(LIBIIO_LIBRARIES LIBIIO_INCLUDE_DIRS)

endif(NOT LIBIIO_FOUND)
