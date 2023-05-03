
cmake_minimum_required (VERSION 2.8.12)

include(ExternalProject)

message( STATUS "CMAKE_SYSTEM: " ${CMAKE_SYSTEM} )
message( STATUS "CMAKE_SOURCE_DIR: " ${CMAKE_SOURCE_DIR} )
message( STATUS "CMAKE_BINARY_DIR: " ${CMAKE_BINARY_DIR} )

message( STATUS "CMAKE_SYSTEM_PROCESSOR: " ${CMAKE_SYSTEM_PROCESSOR} )
message( STATUS "CMAKE_C_COMPILER_TARGET: " ${CMAKE_C_COMPILER_TARGET} )
message( STATUS "CMAKE_CXX_COMPILER_TARGET: " ${CMAKE_CXX_COMPILER_TARGET} )

list(APPEND libusb_configure_cmd
  <SOURCE_DIR>/configure
  CC=${CMAKE_C_COMPILER}
  CFLAGS=-I${CMAKE_INSTALL_PREFIX}/include
  LDFLAGS=-L${CMAKE_INSTALL_PREFIX}/lib
  --prefix=${CMAKE_INSTALL_PREFIX}
  --host=${CMAKE_C_COMPILER_TARGET}
  --disable-udev
)

if(${CMAKE_SYSTEM_PROCESSOR} MATCHES x86)
ExternalProject_Add(libusb
  PREFIX libusb

  URL https://github.com/libusb/libusb/releases/download/v1.0.24/libusb-1.0.24.tar.bz2

  BUILD_IN_SOURCE 1
  UPDATE_COMMAND ""
  PATCH_COMMAND ""
  CONFIGURE_COMMAND COMMAND ${libusb_configure_cmd}
  BUILD_COMMAND make
  INSTALL_COMMAND make install
)
file(GLOB_RECURSE LIBUSB_INSTALL_FILES
  ${CMAKE_INSTALL_PREFIX}/include/libusb*
  ${CMAKE_INSTALL_PREFIX}/lib/libusb*
)
endif()

#[[ - NOT USED
list(APPEND libxml2_configure_cmd
  <SOURCE_DIR>/configure
  CC=${CMAKE_C_COMPILER}
  CFLAGS=-I${CMAKE_INSTALL_PREFIX}/include
  LDFLAGS=-L${CMAKE_INSTALL_PREFIX}/lib
  --prefix=${CMAKE_INSTALL_PREFIX}
  --host=${CMAKE_C_COMPILER_TARGET}
  --without-zlib
  --without-lzma
  --without-iconv
  --without-python
)

ExternalProject_Add(libxml2
  PREFIX libusb

  URL ftp://xmlsoft.org/libxml2/libxml2-2.9.12.tar.gz

  BUILD_IN_SOURCE 1
  UPDATE_COMMAND ""
  PATCH_COMMAND ""
  CONFIGURE_COMMAND COMMAND ${libxml2_configure_cmd}
  BUILD_COMMAND make
  INSTALL_COMMAND make install
)
file(GLOB_RECURSE LIBXML2_INSTALL_FILES
  ${CMAKE_INSTALL_PREFIX}/lib/libxml2*
)
]]

list(APPEND libconfig_configure_cmd
  <SOURCE_DIR>/configure
  CC=${CMAKE_C_COMPILER}
  CFLAGS=-I${CMAKE_INSTALL_PREFIX}/include
  LDFLAGS=-L${CMAKE_INSTALL_PREFIX}/lib
  --prefix=${CMAKE_INSTALL_PREFIX}
  --host=${CMAKE_C_COMPILER_TARGET}
  --disable-cxx
)

ExternalProject_Add(libconfig
  PREFIX libconfig

  URL https://hyperrealm.github.io/libconfig/dist/libconfig-1.7.3.tar.gz

  BUILD_IN_SOURCE 1
  UPDATE_COMMAND ""
  PATCH_COMMAND ""
  CONFIGURE_COMMAND COMMAND ${libconfig_configure_cmd}
  BUILD_COMMAND make
  INSTALL_COMMAND make install
)
file(GLOB_RECURSE LIBCONFIG_INSTALL_FILES
  ${CMAKE_INSTALL_PREFIX}/include/libconfig*
  ${CMAKE_INSTALL_PREFIX}/lib/libconfig*
)

ExternalProject_Add(tracy
  PREFIX tracy

  GIT_REPOSITORY "https://github.com/wolfpld/tracy.git"
  GIT_TAG "v0.9.1"

  UPDATE_COMMAND ""
  PATCH_COMMAND ""
  CONFIGURE_COMMAND ""
  BUILD_COMMAND ""
  INSTALL_COMMAND cp -r <SOURCE_DIR> ${CMAKE_INSTALL_PREFIX}
)
file(GLOB_RECURSE TRACY_INSTALL_FILES
  ${CMAKE_INSTALL_PREFIX}/tracy
)

ExternalProject_Add(libiio
  PREFIX libiio

  GIT_REPOSITORY "https://github.com/analogdevicesinc/libiio.git"
  GIT_TAG "v0.24"

  #PATCH_COMMAND patch < ${CMAKE_SOURCE_DIR}/libiio.patch

  CMAKE_CACHE_ARGS
    -DCMAKE_TOOLCHAIN_FILE:PATH=${CMAKE_TOOLCHAIN_FILE}
    -DCMAKE_INSTALL_PREFIX:PATH=${CMAKE_INSTALL_PREFIX}
    -DCMAKE_POSITION_INDEPENDENT_CODE:BOOL=TRUE
    -DINSTALL_UDEV_RULE:BOOL=FALSE
    -DWITH_TESTS:BOOL=TRUE
    -DHAVE_DNS_SD:BOOL=FALSE
    -DWITH_AIO:BOOL=FALSE
)
file(GLOB_RECURSE LIBIIO_INSTALL_FILES
  ${CMAKE_INSTALL_PREFIX}/include/iio*
  ${CMAKE_INSTALL_PREFIX}/lib/libiio*
  ${CMAKE_INSTALL_PREFIX}/bin/iio*
)

ExternalProject_Add(libad9361-iio
  PREFIX libad9361-iio

  GIT_REPOSITORY "https://github.com/analogdevicesinc/libad9361-iio.git"
  GIT_TAG "43643ab0ef8a3fbdb6c7595f42842f534a291664"

  CMAKE_CACHE_ARGS
    -DCMAKE_TOOLCHAIN_FILE:PATH=${CMAKE_TOOLCHAIN_FILE}
    -DCMAKE_INSTALL_PREFIX:PATH=${CMAKE_INSTALL_PREFIX}
    -DCMAKE_POSITION_INDEPENDENT_CODE:BOOL=TRUE
)
file(GLOB_RECURSE LIBAD9361_IIO_INSTALL_FILES
  ${CMAKE_INSTALL_PREFIX}/include/ad9361*
  ${CMAKE_INSTALL_PREFIX}/lib/libad9361*
)

list(APPEND fftw_configure_cmd
  <SOURCE_DIR>/configure
  "CC=${CMAKE_C_COMPILER} -Ofast"
  CFLAGS=-I${CMAKE_INSTALL_PREFIX}/include
  LDFLAGS=-L${CMAKE_INSTALL_PREFIX}/lib
  --prefix=${CMAKE_INSTALL_PREFIX}
  --host=${CMAKE_C_COMPILER_TARGET}
  --enable-shared=yes
  --enable-single
)

if(${CMAKE_SYSTEM_PROCESSOR} MATCHES armv7l)
list(APPEND fftw_configure_cmd
  --enable-neon
  --with-slow-timer
)
else()
list(APPEND fftw_configure_cmd
  --enable-sse
)
endif()

ExternalProject_Add(fftw
  PREFIX fftw

  URL http://www.fftw.org/fftw-3.3.10.tar.gz

  BUILD_IN_SOURCE 1
  UPDATE_COMMAND ""
  PATCH_COMMAND ""
  CONFIGURE_COMMAND COMMAND ${fftw_configure_cmd}
  BUILD_COMMAND make
  INSTALL_COMMAND make install
)
file(GLOB_RECURSE FFTW_INSTALL_FILES
  ${CMAKE_INSTALL_PREFIX}/include/fftw*
  ${CMAKE_INSTALL_PREFIX}/lib/libfftw*
  ${CMAKE_INSTALL_PREFIX}/bin/fftw*
)

list(APPEND libfec_configure_cmd
  <SOURCE_DIR>/configure
  "CC=${CMAKE_C_COMPILER} -Ofast"
  CFLAGS=-I${CMAKE_INSTALL_PREFIX}/include
  LDFLAGS=-L${CMAKE_INSTALL_PREFIX}/lib
  --prefix=${CMAKE_INSTALL_PREFIX}
  --host=${CMAKE_C_COMPILER_TARGET}
)

ExternalProject_Add(libfec
  PREFIX libfec

  GIT_REPOSITORY "https://github.com/HAMNET-Access-Protocol/libfec.git"
  GIT_TAG "b870f839438bba59dc4981ec0e21c60c4ee6e487"

  BUILD_IN_SOURCE 1
  UPDATE_COMMAND ""
  PATCH_COMMAND sed -i s|ldconfig||g <SOURCE_DIR>/configure
  CONFIGURE_COMMAND COMMAND ${libfec_configure_cmd}
  BUILD_COMMAND make
  INSTALL_COMMAND make install
)
file(GLOB_RECURSE LIBFEC_INSTALL_FILES
  ${CMAKE_INSTALL_PREFIX}/include/fec*
  ${CMAKE_INSTALL_PREFIX}/lib/libfec.*
)

list(APPEND liquid_dsp_configure_cmd
  ac_cv_func_malloc_0_nonnull=yes
  ac_cv_func_realloc_0_nonnull=yes
  <SOURCE_DIR>/configure
  "CC=${CMAKE_C_COMPILER} -Ofast"
  CFLAGS=-I${CMAKE_INSTALL_PREFIX}/include
  LDFLAGS=-L${CMAKE_INSTALL_PREFIX}/lib
  --prefix=${CMAKE_INSTALL_PREFIX}
  --host=${CMAKE_C_COMPILER_TARGET}
)

ExternalProject_Add(liquid-dsp
  PREFIX liquid-dsp

  GIT_REPOSITORY "https://github.com/agrewal707/liquid-dsp.git"
  GIT_TAG "fix_ofdm_equalizer_gain_correction"

  BUILD_IN_SOURCE 1
  BUILD_ALWAYS 1
  UPDATE_COMMAND ""
  PATCH_COMMAND patch -N -p1 < ${CMAKE_SOURCE_DIR}/liquid-dsp.patch || echo "WARNING: PATCH APPLIED?" && exit 0
  CONFIGURE_COMMAND <SOURCE_DIR>/bootstrap.sh COMMAND ${liquid_dsp_configure_cmd}
  BUILD_COMMAND make
  INSTALL_COMMAND make install

  DEPENDS fftw libfec
)
file(GLOB_RECURSE LIQUID_DSP_INSTALL_FILES
  ${CMAKE_INSTALL_PREFIX}/lib/libliquid*
)

ExternalProject_Add(soapysdr
  PREFIX soapysdr

  GIT_REPOSITORY "https://github.com/pothosware/SoapySDR.git"
  GIT_TAG "master"

  PATCH_COMMAND patch -N -p1 < ${CMAKE_SOURCE_DIR}/soapysdr.patch || echo "WARNING: PATCH APPLIED?" && exit 0

  CMAKE_CACHE_ARGS
    -DCMAKE_TOOLCHAIN_FILE:PATH=${CMAKE_TOOLCHAIN_FILE}
    -DCMAKE_INSTALL_PREFIX:PATH=${CMAKE_INSTALL_PREFIX}
)
file(GLOB_RECURSE SOAPYSDR_INSTALL_FILES
  ${CMAKE_INSTALL_PREFIX}/lib/libSoapySDR.*
  ${CMAKE_INSTALL_PREFIX}/bin/SoapySDR*
)

ExternalProject_Add(soapyplutosdr
  PREFIX soapyplutosdr

  GIT_REPOSITORY "https://github.com/agrewal707/SoapyPlutoSDR.git"
  GIT_TAG "wiflx"

  CMAKE_CACHE_ARGS
    -DCMAKE_TOOLCHAIN_FILE:PATH=${CMAKE_TOOLCHAIN_FILE}
    -DCMAKE_INSTALL_PREFIX:PATH=${CMAKE_INSTALL_PREFIX}

  DEPENDS libiio libad9361-iio soapysdr
)

ExternalProject_Add(json
  PREFIX json

  URL "https://github.com/nlohmann/json/archive/v3.11.2.tar.gz"

  CMAKE_CACHE_ARGS
    -DCMAKE_TOOLCHAIN_FILE:PATH=${CMAKE_TOOLCHAIN_FILE}
    -DCMAKE_INSTALL_PREFIX:PATH=${CMAKE_INSTALL_PREFIX}
    -DJSON_BuildTests:BOOL=FALSE
)

if(${CMAKE_SYSTEM_PROCESSOR} MATCHES x86)
ExternalProject_Add(boost
  PREFIX boost
  URL "https://boostorg.jfrog.io/artifactory/main/release/1.76.0/source/boost_1_76_0.tar.bz2"
  URL_HASH SHA256=f0397ba6e982c4450f27bf32a2a83292aba035b827a5623a14636ea583318c41
  BUILD_IN_SOURCE 1
  UPDATE_COMMAND ""
  PATCH_COMMAND echo "using gcc : ${CMAKE_SYSTEM_PROCESSOR} : ${CMAKE_CXX_COMPILER} $<SEMICOLON>" > user-config.jam
  CONFIGURE_COMMAND ./bootstrap.sh --prefix=${CMAKE_INSTALL_PREFIX}
  BUILD_COMMAND ./b2 install
  --toolset=gcc-${CMAKE_SYSTEM_PROCESSOR}
  --user-config=user-config.jam
  --with-system
  --with-date_time
  --with-chrono
  --with-filesystem
  --with-program_options
  --with-serialization
  --with-thread
  --with-test
  --variant=release
  --link=shared
  --runtime-link=shared
  --threading=multi
  INSTALL_COMMAND ""
)
else()
ExternalProject_Add(boost
  PREFIX boost
  URL "https://boostorg.jfrog.io/artifactory/main/release/1.76.0/source/boost_1_76_0.tar.bz2"
  URL_HASH SHA256=f0397ba6e982c4450f27bf32a2a83292aba035b827a5623a14636ea583318c41
  BUILD_IN_SOURCE 1
  UPDATE_COMMAND ""
  PATCH_COMMAND echo "using gcc : ${CMAKE_SYSTEM_PROCESSOR} : ${CMAKE_CXX_COMPILER} $<SEMICOLON>" > user-config.jam
  CONFIGURE_COMMAND ./bootstrap.sh --prefix=${CMAKE_INSTALL_PREFIX}
  BUILD_COMMAND ./b2 install
  --toolset=gcc-${CMAKE_SYSTEM_PROCESSOR}
  --user-config=user-config.jam
  --with-system
  --with-date_time
  --variant=release
  --link=shared
  --runtime-link=shared
  --threading=multi
  INSTALL_COMMAND ""
)
endif()

file(GLOB_RECURSE BOOST_INSTALL_FILES
  ${CMAKE_INSTALL_PREFIX}/lib/libboost*
)

ExternalProject_Add(quill
  PREFIX quill

  GIT_REPOSITORY "https://github.com/odygrd/quill.git"
  GIT_TAG "v2.6.0"

  CMAKE_CACHE_ARGS
    -DCMAKE_TOOLCHAIN_FILE:PATH=${CMAKE_TOOLCHAIN_FILE}
    -DCMAKE_INSTALL_PREFIX:PATH=${CMAKE_INSTALL_PREFIX}
)
file(GLOB_RECURSE QUILL_INSTALL_FILES
  ${CMAKE_INSTALL_PREFIX}/lib/libquill*
)

list(APPEND protobuf_cmake_args
  -DCMAKE_TOOLCHAIN_FILE:PATH=${CMAKE_TOOLCHAIN_FILE}
  -DCMAKE_INSTALL_PREFIX:PATH=${CMAKE_INSTALL_PREFIX}
  -Dprotobuf_BUILD_TESTS:BOOL=OFF
  -Dprotobuf_BUILD_EXAMPLES:BOOL=OFF
  -Dprotobuf_WITH_ZLIB:BOOL=OFF
)

ExternalProject_Add(protobuf
  PREFIX protobuf

  URL "https://github.com/protocolbuffers/protobuf/releases/download/v21.12/protobuf-cpp-3.21.12.tar.gz"

  CMAKE_CACHE_ARGS ${protobuf_cmake_args}

  SOURCE_SUBDIR cmake
)
file(GLOB_RECURSE PROTOBUF_INSTALL_FILES
  ${CMAKE_INSTALL_PREFIX}/bin/protoc*
  ${CMAKE_INSTALL_PREFIX}/lib/libprotoc*
  ${CMAKE_INSTALL_PREFIX}/lib/libprotobuf*
)

if(${CMAKE_SYSTEM_PROCESSOR} MATCHES x86)
ExternalProject_Add(uhd
  PREFIX uhd

  GIT_REPOSITORY "https://github.com/EttusResearch/uhd.git"
  GIT_TAG "v4.4.0.0"

  SOURCE_SUBDIR host

  CMAKE_CACHE_ARGS
    -DCMAKE_TOOLCHAIN_FILE:PATH=${CMAKE_TOOLCHAIN_FILE}
    -DCMAKE_INSTALL_PREFIX:PATH=${CMAKE_INSTALL_PREFIX}
    -DLIBUSB_INCLUDE_DIRS:PATH=${CMAKE_INSTALL_PREFIX}/include/libusb-1.0
    -DLIBUSB_LIBRARIES:PATH=${CMAKE_INSTALL_PREFIX}/lib/libusb-1.0.so

  DEPENDS libusb boost
)
file(GLOB_RECURSE UHD_INSTALL_FILES
  ${CMAKE_INSTALL_PREFIX}/include/uhd.h
  ${CMAKE_INSTALL_PREFIX}/lib/libuhd*
)

ExternalProject_Add(soapyuhd
  PREFIX soapyuhd

  GIT_REPOSITORY "https://github.com/pothosware/SoapyUHD.git"
  GIT_TAG "master"

  CMAKE_CACHE_ARGS
    -DCMAKE_TOOLCHAIN_FILE:PATH=${CMAKE_TOOLCHAIN_FILE}
    -DCMAKE_INSTALL_PREFIX:PATH=${CMAKE_INSTALL_PREFIX}

  DEPENDS uhd soapysdr
)

ExternalProject_Add(bladerf
  PREFIX bladerf

  GIT_REPOSITORY "https://github.com/Nuand/bladeRF.git"
  GIT_TAG "2022.11"

  SOURCE_SUBDIR host

  CMAKE_CACHE_ARGS
    -DCMAKE_TOOLCHAIN_FILE:PATH=${CMAKE_TOOLCHAIN_FILE}
    -DCMAKE_INSTALL_PREFIX:PATH=${CMAKE_INSTALL_PREFIX}
    -DINSTALL_UDEV_RULES:BOOL=FALSE

  DEPENDS libusb
)
file(GLOB_RECURSE BLADERF_INSTALL_FILES
  ${CMAKE_INSTALL_PREFIX}/include/bladeRF*.h
  ${CMAKE_INSTALL_PREFIX}/include/libbladeRF*.h
  ${CMAKE_INSTALL_PREFIX}/libbladeRF*
)

ExternalProject_Add(soapybladerf
  PREFIX soapybladerf

  GIT_REPOSITORY "https://github.com/pothosware/SoapyBladeRF.git"
  GIT_TAG "master"

  CMAKE_CACHE_ARGS
    -DCMAKE_TOOLCHAIN_FILE:PATH=${CMAKE_TOOLCHAIN_FILE}
    -DCMAKE_INSTALL_PREFIX:PATH=${CMAKE_INSTALL_PREFIX}

  DEPENDS bladerf soapysdr
)
endif()


set_property(DIRECTORY PROPERTY ADDITIONAL_MAKE_CLEAN_FILES
  ${LIBUSB_INSTALL_FILES}
  ${CMAKE_INSTALL_PREFIX}/include/libusb-1.0
  ${LIBCONFIG_INSTALL_FILES}
  ${TRACY_INSTALL_FILES}
  ${LIBIIO_INSTALL_FILES}
  ${LIBAD9361_IIO_INSTALL_FILES}
  ${FFTW_INSTALL_FILES}
  ${LIBFEC_INSTALL_FILES}
  ${UHD_INSTALL_FILES}
  ${CMAKE_INSTALL_PREFIX}/include/uhd
  ${CMAKE_INSTALL_PREFIX}/lib/uhd
  ${BLADERF_INSTALL_FILES}
  ${CMAKE_INSTALL_PREFIX}/include/liquid
  ${LIQUID_DSP_INSTALL_FILES}
  ${CMAKE_INSTALL_PREFIX}/include/SoapySDR
  ${CMAKE_INSTALL_PREFIX}/lib/SoapySDR
  ${SOAPYSDR_INSTALL_FILES}
  ${CMAKE_INSTALL_PREFIX}/include/nlohmann
  ${CMAKE_INSTALL_PREFIX}/include/boost
  ${BOOST_INSTALL_FILES}
  ${CMAKE_INSTALL_PREFIX}/include/quill
  ${QUILL_INSTALL_FILES}
  ${CMAKE_INSTALL_PREFIX}/include/google
  ${PROTOBUF_INSTALL_FILES}
  ${CMAKE_INSTALL_PREFIX}/lib/cmake
  ${CMAKE_INSTALL_PREFIX}/lib/pkgconfig
  ${CMAKE_INSTALL_PREFIX}/tracy
)
