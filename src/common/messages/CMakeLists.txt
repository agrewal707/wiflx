#
# CMake configuration file for wiflx protobuf messages
#

# proto files
file(GLOB ProtoFiles "${CMAKE_CURRENT_SOURCE_DIR}/*.proto")

# compile flags
add_definitions (-fPIC)

add_library(${PROJECT_NAME}_messages STATIC ${ProtoFiles})

# Generate code in build tree
protobuf_generate(
  LANGUAGE cpp
  TARGET ${PROJECT_NAME}_messages
)

target_include_directories(${PROJECT_NAME}_messages PUBLIC ${CMAKE_CURRENT_BINARY_DIR})
