#[[
Copyright 2021, Yang Luo"
This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.

@Author
Yang Luo, PHD
Shenyang Institute of Automation, Chinese Academy of Sciences.
email: luoyang@sia.cn

@Created on: 2021.12.06
@Last modified: 2023.04.10
]]

cmake_minimum_required(VERSION 3.10)
project(rocos_app VERSION 0.1.0 DESCRIPTION "ROCOS Application Library")

list(APPEND CMAKE_PREFIX_PATH "/opt/rocos/ecm")
list(APPEND CMAKE_PREFIX_PATH "/opt/grpc")
include(cmake/common.cmake) # gRPC

# add_compile_options(-std=c++11)
set(CMAKE_CXX_STANDARD 11)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

# rocos-app install path
include(GNUInstallDirs)
set(CMAKE_INSTALL_PREFIX /opt/rocos/app)
message(STATUS "INSTALL PREFIX: ${CMAKE_INSTALL_PREFIX}")
#message(STATUS "INSTALL INCLUDE DIR: ${CMAKE_INSTALL_INCLUDEDIR}")
#message(STATUS "INSTALL LIB DIR: ${CMAKE_INSTALL_LIBDIR}")
#message(STATUS "INSTALL BIN DIR: ${CMAKE_INSTALL_BINDIR}")

set(${PROJECT_NAME}_INSTALL_CONFIGDIR ${CMAKE_INSTALL_PREFIX}/config)
set(${PROJECT_NAME}_INSTALL_CMAKEDIR ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME})


##################
## Dependencies ##
##################
# find_package(Boost REQUIRED COMPONENTS system)
find_package(Boost REQUIRED COMPONENTS date_time filesystem system chrono thread)
find_package(yaml-cpp REQUIRED)
find_package(Threads REQUIRED)
#find_package(Qt5 REQUIRED COMPONENTS Core)
find_package(Eigen3 REQUIRED)
find_package(OpenSSL REQUIRED)

find_package(rocos_ecm REQUIRED)

##################
##   3rdparty   ##
##################
add_subdirectory(3rdparty/kdl)
add_subdirectory(3rdparty/nlopt)
add_subdirectory(3rdparty/trac_ik)
add_subdirectory(3rdparty/kdl_parser)
add_subdirectory(3rdparty/ruckig)
add_subdirectory(3rdparty/plog)
add_subdirectory(3rdparty/gripper)


###########
## Build ##
###########

include_directories(
        include
        include/${PROJECT_NAME}
        include/sdk
)

link_directories(
        lib
)

file(GLOB_RECURSE SRCS src/*.cpp)
file(GLOB_RECURSE HDRS include/*.h include/*.hpp)
file(GLOB_RECURSE CONFIG_FILE config/*.urdf  config/public.key  config/license )

set(DEP_LIBS
        dl
        rt
        Threads::Threads
        Boost::boost
        Boost::date_time
        Boost::filesystem
        Boost::system
        Boost::chrono
        Boost::thread
        yaml-cpp

        )

######### Library Hardware ###############
add_library(hardware
        SHARED
        src/command.cc
        src/status_word.cc
        src/control_word.cc
        src/drive_state.cc
        src/hardware.cc
        src/hardware_interface.cc
        src/hardware_sim.cc
        )
target_link_libraries(hardware
        PUBLIC
#        ${DEP_LIBS}
        Boost::thread
        rocos::ecat_config
        )
add_library(rocos::hardware ALIAS hardware)


########## Library Drive #################
add_library(drive
        SHARED
        src/drive.cc
        src/drive_guard.cc
        src/interpolate.cc
        src/robotMath.cc
        )
target_link_libraries(drive
        PUBLIC
        rocos::hardware
        )
add_library(rocos::drive ALIAS drive)

########## Library kinematics #################
add_library(kinematics
        SHARED
        src/kinematics.cc
        )
target_include_directories(kinematics
        PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
        )

target_link_libraries(kinematics
        PUBLIC
        Eigen3::Eigen
        nlopt
        orocos-kdl
        $<BUILD_INTERFACE:trac_ik>
        ruckig::ruckig
        )
add_library(rocos::kinematics ALIAS kinematics)

########## Library robot #################
add_library(robot
        SHARED
        src/robot.cc
        src/JC_helper_kinematics.cpp
        src/JC_helper_dynamics.cpp
        src/JC_helper_authenticate.cpp
        )
target_link_libraries(robot
        PUBLIC
        rocos::hardware
        rocos::drive
        rocos::kinematics
        $<BUILD_INTERFACE:plog::plog>
        $<BUILD_INTERFACE:JC_helper::gripper>
        OpenSSL::SSL
        )
add_library(rocos::robot ALIAS robot)


################
##    gRPC    ##
################

file(GLOB PROTO_FILES protos/*.proto) # protobuf
file(GLOB GRPC_FILES protos/*_service.proto) # gRPC

foreach (PFIL ${PROTO_FILES}) #protobuf gen
    # Proto file
    get_filename_component(PROTO_ABS ${PFIL} ABSOLUTE)
    get_filename_component(PROTO_PATH ${PFIL} PATH)
    get_filename_component(PROTO_NAME ${PFIL} NAME_WE)

    # Generated sources
    set(PROTO_SRC "${CMAKE_BINARY_DIR}/${PROTO_NAME}.pb.cc")
    set(PROTO_HDR "${CMAKE_BINARY_DIR}/${PROTO_NAME}.pb.h")
    add_custom_command(
            OUTPUT "${PROTO_SRC}" "${PROTO_HDR}"
            COMMAND ${_PROTOBUF_PROTOC}
            ARGS --cpp_out "${CMAKE_BINARY_DIR}"
            -I "${PROTO_PATH}"
            "${PROTO_ABS}"
            DEPENDS "${PROTO_ABS}"
    )
    list(APPEND PROTO_SRCS ${PROTO_SRC})
    list(APPEND PROTO_HDRS ${PROTO_HDR})
endforeach ()

foreach (GFIL ${GRPC_FILES}) #grpc gen
    # Proto file
    get_filename_component(GRPC_ABS ${GFIL} ABSOLUTE)
    get_filename_component(GRPC_PATH ${GFIL} PATH)
    get_filename_component(GRPC_NAME ${GFIL} NAME_WE)

    # Generated sources
    set(GRPC_SRC "${CMAKE_BINARY_DIR}/${GRPC_NAME}.grpc.pb.cc")
    set(GRPC_HDR "${CMAKE_BINARY_DIR}/${GRPC_NAME}.grpc.pb.h")
    add_custom_command(
            OUTPUT "${GRPC_SRC}" "${GRPC_HDR}"
            COMMAND ${_PROTOBUF_PROTOC}
            ARGS --grpc_out "${CMAKE_BINARY_DIR}"
            -I "${GRPC_PATH}"
            --plugin=protoc-gen-grpc="${_GRPC_CPP_PLUGIN_EXECUTABLE}"
            "${GRPC_ABS}"
            DEPENDS "${GRPC_ABS}"
    )
    list(APPEND GRPC_SRCS ${GRPC_SRC})
    list(APPEND GRPC_HDRS ${GRPC_HDR})
endforeach ()

# Include generated *.pb.h files
include_directories("${CMAKE_CURRENT_BINARY_DIR}")

########## Library protocol #################

add_library(protocol
        SHARED
        ${PROTO_SRCS}
        ${PROTO_HDRS}
        ${GRPC_SRCS}
        ${GRPC_HRDS}
        src/robot_service.cc
        )
target_link_libraries(protocol
        PUBLIC
        ${_REFLECTION}
        ${_GRPC_GRPCPP}
        ${_PROTOBUF_LIBPROTOBUF}
        rocos::robot
        )
add_library(rocos::protocol ALIAS protocol)


add_library(rocos_app
        INTERFACE)
target_link_libraries(rocos_app
        INTERFACE
        rocos::hardware
        rocos::drive
        rocos::protocol
        rocos::kinematics
        rocos::robot
        )
add_library(rocos::rocos_app ALIAS rocos_app)

########## Executable RocosAppMain ################
# move rocosAppMain to test. Due to rocos_app is a robot library.
#add_executable(${PROJECT_NAME}
#        src/rocosAppMain.cc
#        )
#
### Specify libraries to link a library or executable target against
#target_link_libraries(${PROJECT_NAME}
#        PRIVATE
#        rocos::hardware
#        rocos::drive
#        rocos::protocol
#        rocos::kinematics
#        rocos::robot
#        )

configure_file(include/ver.h.in ver.h) # Version Definition

# Copy config files to bin/
file(COPY ${CONFIG_FILE} DESTINATION ${CMAKE_BINARY_DIR}/bin)

#############
## Install ##
#############

# Add support for installation
include(CMakePackageConfigHelpers)

install(FILES ${CMAKE_BINARY_DIR}/ver.h
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/rocos_app
        )

# Install hardware library
install(TARGETS hardware
        EXPORT hardware-targets
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}  # 动态库安装路径
        ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}  # 静态库安装路径
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}  # 可执行文件安装路径
        PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}  # 头文件安装路径
        )

# Install drive library
install(TARGETS drive
        EXPORT drive-targets
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}  # 动态库安装路径
        ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}  # 静态库安装路径
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}  # 可执行文件安装路径
        PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}  # 头文件安装路径
        )

# Install kinematics library
install(TARGETS kinematics
        EXPORT kinematics-targets
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}  # 动态库安装路径
        ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}  # 静态库安装路径
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}  # 可执行文件安装路径
        PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}  # 头文件安装路径
        )

# Install protocol library
install(TARGETS protocol
        EXPORT protocol-targets
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}  # 动态库安装路径
        ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}  # 静态库安装路径
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}  # 可执行文件安装路径
        PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}  # 头文件安装路径
        )

# Install robot library
install(TARGETS robot
        EXPORT robot-targets
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}  # 动态库安装路径
        ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}  # 静态库安装路径
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}  # 可执行文件安装路径
        PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}  # 头文件安装路径
        )

install(TARGETS rocos_app
        EXPORT rocos_app-targets
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}  # 动态库安装路径
        ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}  # 静态库安装路径
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}  # 可执行文件安装路径
        PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}  # 头文件安装路径
        )

# Install config files
install(FILES ${CONFIG_FILE}
        DESTINATION ${${PROJECT_NAME}_INSTALL_CONFIGDIR}
        )

install(EXPORT hardware-targets
        FILE hardware-targets.cmake
        NAMESPACE rocos::
        DESTINATION ${${PROJECT_NAME}_INSTALL_CMAKEDIR}
        )

install(EXPORT drive-targets
        FILE drive-targets.cmake
        NAMESPACE rocos::
        DESTINATION ${${PROJECT_NAME}_INSTALL_CMAKEDIR}
        )

install(EXPORT kinematics-targets
        FILE kinematics-targets.cmake
        NAMESPACE rocos::
        DESTINATION ${${PROJECT_NAME}_INSTALL_CMAKEDIR}
        )

install(EXPORT protocol-targets
        FILE protocol-targets.cmake
        NAMESPACE rocos::
        DESTINATION ${${PROJECT_NAME}_INSTALL_CMAKEDIR}
        )

install(EXPORT robot-targets
        FILE robot-targets.cmake
        NAMESPACE rocos::
        DESTINATION ${${PROJECT_NAME}_INSTALL_CMAKEDIR}
        )

install(EXPORT rocos_app-targets
        FILE rocos_app-targets.cmake
        NAMESPACE rocos::
        DESTINATION ${${PROJECT_NAME}_INSTALL_CMAKEDIR}
        )

configure_package_config_file(
        cmake/rocos_app-config.cmake.in
        rocos_app-config.cmake
        INSTALL_DESTINATION ${${PROJECT_NAME}_INSTALL_CMAKEDIR}
)

# 写入库的版本信息
write_basic_package_version_file(
        rocos_app-config-version.cmake
        VERSION ${PACKAGE_VERSION}
        COMPATIBILITY AnyNewerVersion  # 表示该函数库向下兼容
)

install(FILES "${CMAKE_CURRENT_BINARY_DIR}/rocos_app-config.cmake"
        "${CMAKE_CURRENT_BINARY_DIR}/rocos_app-config-version.cmake"
        DESTINATION ${${PROJECT_NAME}_INSTALL_CMAKEDIR}
        )


#############
## Testing ##
#############

enable_testing()
add_executable(unit_test test/unit_test.cc)
target_link_libraries(unit_test
        PRIVATE
        rocos::hardware
        rocos::drive
        rocos::protocol
        rocos::kinematics
        rocos::robot
        )
add_executable(main_test test/main_test.cc)
target_link_libraries(main_test
        PRIVATE
        rocos::hardware
        rocos::drive
        rocos::protocol
        rocos::kinematics
        rocos::robot
        )
add_test(NAME unit_test COMMAND unit_test)
