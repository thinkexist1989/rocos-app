cmake_minimum_required(VERSION 3.10)
project(rocos_app VERSION 1.0)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_PREFIX_PATH "/opt/grpc")
set(CMAKE_CXX_STANDARD 17)


include(lib/cmake/common.cmake) # gRPC
include_directories("${CMAKE_CURRENT_SOURCE_DIR}")

# ##############################################################################
# Dependencies ##
# ##############################################################################
find_package(robot_export REQUIRED PATHS lib/cmake/robot_export)

# ######### Executable RocosAppMain ################
add_executable(rocos_app rocosAppMain.cc)

# Specify libraries to link a library or executable target against
target_link_libraries(  rocos_app PRIVATE rocos::protocol rocos::robot console_bridge::console_bridge OpenSSL::SSL
)


# Copy config files to bin/
file(GLOB_RECURSE CONFIG_FILE config/*.urdf config/public.key config/license)
file(COPY ${CONFIG_FILE} DESTINATION ${CMAKE_BINARY_DIR}/bin)


