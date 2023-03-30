# dependencies
find_package(liquid-dsp REQUIRED)
find_package(libiio REQUIRED)

# build flags
add_definitions (-Werror -Wno-deprecated -Wno-deprecated-declarations)

message (STATUS "CMAKE_BUILD_TYPE: " ${CMAKE_BUILD_TYPE})
#message (STATUS "CMAKE_C_FLAGS: " ${CMAKE_C_FLAGS_RELEASE})
#message (STATUS "CMAKE_CXX_FLAGS: " ${CMAKE_CXX_FLAGS_RELEASE})
#message (STATUS "CMAKE_EXE_LINKER_FLAGS: " ${CMAKE_EXE_LINKER_FLAGS_RELEASE})
#message (STATUS "CMAKE_SHARED_LINKER_FLAGS: " ${CMAKE_SHARED_LINKER_FLAGS_RELEASE})

include_directories(${LIQUID_DSP_INCLUDE_DIRS})
link_directories(${LIQUID_DSP_LIBRARY_DIRS})

# header paths
include_directories(
	${CMAKE_INSTALL_PREFIX}/include
)

# library paths
link_directories(
	${CMAKE_INSTALL_PREFIX}/lib
)

# build
add_executable (ad9361_iiostream ad9361-iiostream.c cw.c sc_framegen.c sc_framesync.c sc_flexframegen.c sc_flexframesync.c sc_qdetector_cccf.c sc_common.c)
target_link_libraries (ad9361_iiostream
  ${LIQUID_DSP_STATIC_LIBRARIES}
  ${LIBFEC_STATIC_LIBRARIES}
  ${FFTW3F_STATIC_LIBRARIES}
  -liio
  -lad9361
  -lm
  -lpthread
  )

# install
install (TARGETS ad9361_iiostream DESTINATION bin)
