cmake_minimum_required(VERSION 3.10)
project(rocos_app VERSION 1.0)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_PREFIX_PATH "/opt/grpc")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE Release)


include(cmake/common.cmake) # gRPC
include_directories("${CMAKE_CURRENT_SOURCE_DIR}")

# ##############################################################################
# Dependencies ##
# ##############################################################################
find_package(robot_export REQUIRED PATHS lib/cmake/robot_export)


# ##############################################################################
# gRPC    ##
# ##############################################################################
file(GLOB PROTO_FILES protos/*.proto) # protobuf
file(GLOB GRPC_FILES protos/*_service.proto) # gRPC

foreach(PFIL ${PROTO_FILES}) # protobuf gen
  # Proto file
  get_filename_component(PROTO_ABS ${PFIL} ABSOLUTE)
  get_filename_component(PROTO_PATH ${PFIL} PATH)
  get_filename_component(PROTO_NAME ${PFIL} NAME_WE)

  # Generated sources
  set(PROTO_SRC "${CMAKE_BINARY_DIR}/${PROTO_NAME}.pb.cc")
  set(PROTO_HDR "${CMAKE_BINARY_DIR}/${PROTO_NAME}.pb.h")
  add_custom_command(
    OUTPUT "${PROTO_SRC}" "${PROTO_HDR}"
    COMMAND ${_PROTOBUF_PROTOC} ARGS --cpp_out "${CMAKE_BINARY_DIR}/" -I
    "${PROTO_PATH}" "${PROTO_ABS}"
    DEPENDS "${PROTO_ABS}")
  list(APPEND PROTO_SRCS ${PROTO_SRC})
  list(APPEND PROTO_HDRS ${PROTO_HDR})
endforeach()

foreach(GFIL ${GRPC_FILES}) # grpc gen
  # Proto file
  get_filename_component(GRPC_ABS ${GFIL} ABSOLUTE)
  get_filename_component(GRPC_PATH ${GFIL} PATH)
  get_filename_component(GRPC_NAME ${GFIL} NAME_WE)

  # Generated sources
  set(GRPC_SRC "${CMAKE_BINARY_DIR}/${GRPC_NAME}.grpc.pb.cc")
  set(GRPC_HDR "${CMAKE_BINARY_DIR}/${GRPC_NAME}.grpc.pb.h")
  add_custom_command(
    OUTPUT "${GRPC_SRC}" "${GRPC_HDR}"
    COMMAND
    ${_PROTOBUF_PROTOC} ARGS --grpc_out "${CMAKE_BINARY_DIR}/" -I
    "${GRPC_PATH}" --plugin=protoc-gen-grpc="${_GRPC_CPP_PLUGIN_EXECUTABLE}"
    "${GRPC_ABS}"
    DEPENDS "${GRPC_ABS}")
  list(APPEND GRPC_SRCS ${GRPC_SRC})
  list(APPEND GRPC_HDRS ${GRPC_HDR})
endforeach()

include_directories("${CMAKE_CURRENT_BINARY_DIR}")

# ######### Library protocol #################
add_library(protocol SHARED ${PROTO_SRCS} ${PROTO_HDRS} ${GRPC_SRCS}
  ${GRPC_HRDS} src/robot_service.cc)
target_link_libraries(protocol PUBLIC ${_REFLECTION} ${_GRPC_GRPCPP}
  ${_PROTOBUF_LIBPROTOBUF} rocos::robot)
add_library(rocos::protocol ALIAS protocol)
##################################################



# ######### Executable RocosAppMain ################
add_executable(rocos_app src/rocosAppMain.cc)

# Specify libraries to link a library or executable target against
target_link_libraries(  rocos_app PRIVATE rocos::protocol  console_bridge::console_bridge OpenSSL::SSL
)


# Copy config files to bin/
file(GLOB_RECURSE CONFIG_FILE config/*.urdf config/public.key config/license)
file(COPY ${CONFIG_FILE} DESTINATION ${CMAKE_BINARY_DIR}/bin)


