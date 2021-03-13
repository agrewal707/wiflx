find_package(liquid-dsp REQUIRED)
find_package(libiio REQUIRED)
find_package(libfec REQUIRED)
find_package(fftw3f REQUIRED)
find_package(soapysdr REQUIRED)
find_package (Protobuf 3.14.0 REQUIRED PATHS ${CMAKE_INSTALL_PREFIX}/lib/cmake)
set (Boost_NO_SYSTEM_PATHS       ON)
set(Boost_USE_STATIC_LIBS        ON)  # only find static libs
set(Boost_USE_DEBUG_LIBS         OFF) # ignore debug libs and
set(Boost_USE_RELEASE_LIBS       ON)  # only find release libs
set(Boost_USE_MULTITHREADED      ON)
set(Boost_USE_STATIC_RUNTIME     OFF)
cmake_policy(SET CMP0057 NEW)
message (STATUS "PATH PATH" ${CMAKE_INSTALL_PREFIX}/lib/cmake)
find_package (Boost 1.74.0 REQUIRED system PATHS ${CMAKE_INSTALL_PREFIX}/lib/cmake)

include_directories(${LIQUID_DSP_INCLUDE_DIRS})
link_directories(${LIQUID_DSP_LIBRARY_DIRS})

include_directories(${LIBFEC_INCLUDE_DIRS})
link_directories(${LIBFEC_LIBRARY_DIRS})

# build flags
add_definitions (-std=c++11 -std=c++14 -std=c++17)
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -DWIFLX_TRACE")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -fvisibility-inlines-hidden")
set(CMAKE_CXX_FLAGS_MINSIZEREL "${CMAKE_CXX_FLAGS_MINSIZEREL} -fvisibility-inlines-hidden")

# header paths
include_directories(
  ${PROJECT_SOURCE_DIR}/src
	${PROJECT_BINARY_DIR}/src
	${CMAKE_INSTALL_PREFIX}/include
)

# library paths
link_directories(
	${CMAKE_INSTALL_PREFIX}/lib
)

# internal libraries
add_subdirectory (messages)


# header paths
include_directories(
	${PROJECT_BINARY_DIR}
  ${PROJECT_SOURCE_DIR}
	${Boost_INCLUDE_DIRS}
)

# library paths
link_directories(
  ${Boost_LIBRARY_DIRS}
)

# build
file (GLOB WIFLX_COMMON_SOURCES *.cpp)

add_library(${PROJECT_NAME}_common STATIC ${WIFLX_COMMON_SOURCES})
target_include_directories(${PROJECT_NAME}_common PUBLIC ${CMAKE_CURRENT_BINARY_DIR})
#target_link_libraries (${PROJECT_NAME}_common ${PROJECT_NAME}_messages)
