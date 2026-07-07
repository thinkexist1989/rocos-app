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

cmake_minimum_required(VERSION 3.15)
project(rocos_app VERSION 2.0.0 DESCRIPTION "ROCOS Application Library")

# list(APPEND CMAKE_PREFIX_PATH "/opt/rocos/ecm")
# add_compile_options(-std=c++11)
set(CMAKE_CXX_STANDARD 17)

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
find_package(Eigen3 REQUIRED)

# find_package(rocos_ecm REQUIRED)

find_package(orocos_kdl REQUIRED)

##################
##   3rdparty   ##
##################
add_subdirectory(3rdparty/nlopt)
add_subdirectory(3rdparty/trac_ik)
add_subdirectory(3rdparty/kdl_parser)
add_subdirectory(3rdparty/ruckig)
add_subdirectory(3rdparty/plog)
#add_subdirectory(3rdparty/gripper)
add_subdirectory(3rdparty/gflags)
add_subdirectory(3rdparty/spdlog)
add_subdirectory(3rdparty/sml)


###########
## Build ##
###########

include_directories(
        include
)

link_directories(
        lib
)

file(GLOB_RECURSE SRCS src/*.cpp)
file(GLOB_RECURSE HDRS include/*.h include/*.hpp)
file(GLOB_RECURSE CONFIG_FILE config/*.urdf config/*.yaml )

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
        Eigen3::Eigen
        )


add_library(logger SHARED src/logger.cpp)
target_link_libraries(logger PUBLIC spdlog::spdlog_header_only)
add_library(rocos::logger ALIAS logger)

######### Library Hardware ###############
#add_library(hardware
#        SHARED
#        src/hardware.cpp
#
#        )
#target_link_libraries(hardware
#        PUBLIC
#        ${DEP_LIBS}
#        )
#add_library(rocos::hardware ALIAS hardware)

########## Library Drive #################
#add_library(drive
#        SHARED
#        src/drive.cpp
#        src/drive_guard.cpp
#)
#target_link_libraries(drive
#        PUBLIC
#        $<BUILD_INTERFACE:rocos::hardware>
#        )
#add_library(rocos::drive ALIAS drive)


########## Library robot #################
add_library(robot
        SHARED
        src/hardware.cpp
        src/executor.cpp
        src/robot.cpp
        src/model.cpp

        src/move_joint.cpp
        src/move_line.cpp
        src/move_circle.cpp

        src/position_controller.cpp
        src/joint_impedance_controller.cpp
        src/cartesian_impedance_controller.cpp
        src/unit_velocity_profile.cpp

        src/virtual_wall.cpp
        src/virtual_wall_guard.cpp

        src/performance_profiler.cpp
)
target_include_directories(robot
        PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/httplib>
        $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/3rdparty/json>
        $<INSTALL_INTERFACE:include>
        )
target_link_libraries(robot
        PUBLIC
        ${DEP_LIBS}
        $<BUILD_INTERFACE:rocos::logger>
        $<BUILD_INTERFACE:ruckig::ruckig>
        $<BUILD_INTERFACE:trac_ik>
        sml::sml
        orocos-kdl
        )
add_library(rocos::robot ALIAS robot)


# rocos_app convenience target (no protocol - replaced by HTTP server in robot library)
add_library(rocos_app INTERFACE)
target_link_libraries(rocos_app
        INTERFACE
        robot
        )
add_library(rocos::rocos_app ALIAS rocos_app)

########### Executable RocosAppMain ################
# move rocosAppMain to test. Due to rocos_app is a robot library.
add_executable(rocosAppMain
        src/rocosAppMain.cpp
        )

## Specify libraries to link a library or executable target against
target_link_libraries(rocosAppMain
        PRIVATE
        robot
        gflags::gflags
        pthread
        )

########### Executable VirtualWallMain ################
add_executable(VirtualWallMain
        src/VirtualWallMain.cpp
        )

target_link_libraries(VirtualWallMain
        PRIVATE
        robot
        pthread
        )


configure_file(include/ver.h.in ver.h) # Version Definition

# Copy config files to bin/
file(COPY ${CONFIG_FILE} DESTINATION ${CMAKE_BINARY_DIR}/bin)

#############
## Install ##
#############

# Add support for installation
#include(CMakePackageConfigHelpers)
#
#install(FILES ${CMAKE_BINARY_DIR}/ver.h
#        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/rocos_app
#        )
#
#install(DIRECTORY include/rocos_app
#        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
#        )
#
#install(TARGETS hardware drive robot rocos_app rocosAppMain
#        EXPORT rocos_app-targets
#        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}  # 动态库安装路径
#        ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}  # 静态库安装路径
#        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}  # 可执行文件安装路径
#        PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}  # 头文件安装路径
#        )
#
## Install config files
#install(FILES ${CONFIG_FILE}
#        DESTINATION ${${PROJECT_NAME}_INSTALL_CONFIGDIR}
#        )
#
#install(EXPORT rocos_app-targets
#        FILE rocos_app-targets.cmake
#        NAMESPACE rocos::
#        DESTINATION ${${PROJECT_NAME}_INSTALL_CMAKEDIR}
#        )
#
#
#configure_package_config_file(
#        cmake/rocos_app-config.cmake.in
#        rocos_app-config.cmake
#        INSTALL_DESTINATION ${${PROJECT_NAME}_INSTALL_CMAKEDIR}
#)
#
## 写入库的版本信息
#write_basic_package_version_file(
#        rocos_app-config-version.cmake
#        VERSION ${PACKAGE_VERSION}
#        COMPATIBILITY AnyNewerVersion  # 表示该函数库向下兼容
#)
#
#install(FILES "${CMAKE_CURRENT_BINARY_DIR}/rocos_app-config.cmake"
#        "${CMAKE_CURRENT_BINARY_DIR}/rocos_app-config-version.cmake"
#        DESTINATION ${${PROJECT_NAME}_INSTALL_CMAKEDIR}
#        )


#############
## Testing ##
#############

enable_testing()
add_executable(model_test test/model_test.cc)
target_link_libraries(model_test
        PRIVATE
        robot
        )
add_test(NAME model_test COMMAND unit_test)
set_tests_properties(model_test PROPERTIES
        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR})

add_executable(robot_fsm_test test/robot_fsm_test.cc)
target_link_libraries(robot_fsm_test
        PRIVATE
        robot
        )
add_test(NAME robot_fsm_test COMMAND robot_fsm_test)
set_tests_properties(robot_fsm_test PROPERTIES
        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR})

add_executable(hardware_test test/hardware_test.cc)
target_link_libraries(hardware_test
        PRIVATE
        robot
        )
add_test(NAME hardware_test COMMAND hardware_test)
set_tests_properties(hardware_test PROPERTIES
        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR})

add_executable(hardware_enable_test test/hardware_enable_test.cc)
target_link_libraries(hardware_enable_test
        PRIVATE
        robot
        )
# 注意：本测试依赖真实 EtherCAT 硬件，仅在连接机械臂时手动运行
# 使用方法: ./build/bin/hardware_enable_test [ecat_id] [config_path]

add_executable(position_controller_test test/position_controller_test.cc)
target_link_libraries(position_controller_test
        PRIVATE
        robot
        )
add_test(NAME position_controller_test COMMAND position_controller_test)
set_tests_properties(position_controller_test PROPERTIES
        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR})

add_executable(joint_admittance_controller_test test/joint_admittance_controller_test.cc)
target_link_libraries(joint_admittance_controller_test
        PRIVATE
        robot
        )
add_test(NAME joint_admittance_controller_test COMMAND joint_admittance_controller_test)
set_tests_properties(joint_admittance_controller_test PROPERTIES
        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR})

add_executable(joint_impedance_controller_test test/joint_impedance_controller_test.cc)
target_link_libraries(joint_impedance_controller_test
        PRIVATE
        robot
        )
add_test(NAME joint_impedance_controller_test COMMAND joint_impedance_controller_test)
set_tests_properties(joint_impedance_controller_test PROPERTIES
        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR})

add_executable(unit_velocity_profile_test test/unit_velocity_profile_test.cc)
target_link_libraries(unit_velocity_profile_test
        PRIVATE
        robot
        )
add_test(NAME unit_velocity_profile_test COMMAND unit_velocity_profile_test)
set_tests_properties(unit_velocity_profile_test PROPERTIES
        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR})

add_executable(performance_profiler_test test/performance_profiler_test.cc)
target_link_libraries(performance_profiler_test
        PRIVATE
        robot
        )
add_test(NAME performance_profiler_test COMMAND performance_profiler_test)
set_tests_properties(performance_profiler_test PROPERTIES
        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR})

add_executable(move_joint_test test/move_joint_test.cc)
target_link_libraries(move_joint_test
        PRIVATE
        robot
        )
add_test(NAME move_joint_test COMMAND move_joint_test)
set_tests_properties(move_joint_test PROPERTIES
        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR})

add_executable(move_line_test test/move_line_test.cc)
target_link_libraries(move_line_test
        PRIVATE
        robot
        )
add_test(NAME move_line_test COMMAND move_line_test)
set_tests_properties(move_line_test PROPERTIES
        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR})

add_executable(move_circle_test test/move_circle_test.cc)
target_link_libraries(move_circle_test
        PRIVATE
        robot
        )
add_test(NAME move_circle_test COMMAND move_circle_test)
set_tests_properties(move_circle_test PROPERTIES
        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR})

add_executable(move_circle_three_point_test test/move_circle_three_point_test.cc)
target_link_libraries(move_circle_three_point_test
        PRIVATE
        robot
        )
add_test(NAME move_circle_three_point_test COMMAND move_circle_three_point_test)
set_tests_properties(move_circle_three_point_test PROPERTIES
        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR})

add_executable(virtual_wall_test test/virtual_wall_test.cc)
target_link_libraries(virtual_wall_test
        PRIVATE
        robot
        )
add_test(NAME virtual_wall_test COMMAND virtual_wall_test)
set_tests_properties(virtual_wall_test PROPERTIES
        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR})
