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
]]

cmake_minimum_required(VERSION 3.10)
project(rocos_app VERSION 1.9.2)

set(CMAKE_PREFIX_PATH "/opt/grpc")
include(cmake/common.cmake) # gRPC

# add_compile_options(-std=c++11)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_INSTALL_PREFIX ${CMAKE_CURRENT_SOURCE_DIR}/example)


set(CMAKE_INSTALL_RPATH "../../lib")
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
set(CMAKE_BUILD_TYPE Release)


# ##############################################################################
# Dependencies ##
# ##############################################################################
# find_package(Boost REQUIRED COMPONENTS system)
find_package(Boost REQUIRED COMPONENTS date_time filesystem system chrono thread)
find_package(yaml-cpp REQUIRED)
find_package(Threads REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Core)
find_package(Eigen3 REQUIRED)
find_package(OpenSSL REQUIRED)

# ##############################################################################
# 3rdparty   ##
# ##############################################################################
add_subdirectory(3rdparty/kdl)
add_subdirectory(3rdparty/nlopt)
add_subdirectory(3rdparty/trac_ik)
add_subdirectory(3rdparty/kdl_parser)
add_subdirectory(3rdparty/ruckig)
add_subdirectory(3rdparty/plog)
add_subdirectory(3rdparty/gripper)

# ##############################################################################
# Build ##
# ##############################################################################
include_directories(include)



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
  nlopt
  orocos-kdl
  trac_ik
  ruckig::ruckig
  plog::plog
  OpenSSL::SSL
  JC_helper::gripper
  z
)

# Library Hardware ###############
add_library(
  hardware SHARED
  src/command.cc
  src/status_word.cc
  src/control_word.cc
  src/drive_state.cc
  src/hardware.cc
  src/hardware_interface.cc
  src/hardware_sim.cc)
target_link_libraries(hardware PUBLIC ${DEP_LIBS})
add_library(rocos::hardware ALIAS hardware)
##################################################


# ######### Library Drive #################
add_library(drive SHARED src/drive.cc src/drive_guard.cc src/interpolate.cc
  src/robotMath.cc)
target_link_libraries(drive PUBLIC rocos::hardware)
add_library(rocos::drive ALIAS drive)
##################################################


# ######### Library kinematics #################
add_library(kinematics SHARED src/kinematics.cc)
target_link_libraries(kinematics PUBLIC rocos::drive)
add_library(rocos::kinematics ALIAS kinematics)
##################################################



# ######### Library robot #################
add_library(
  robot SHARED src/robot.cc src/JC_helper_kinematics.cpp
  src/JC_helper_dynamics.cpp src/JC_helper_authenticate.cpp)
target_link_libraries(robot PUBLIC rocos::kinematics  )
target_include_directories(robot
PUBLIC
  $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
add_library(rocos::robot ALIAS robot)
##################################################



# ##############################################################################
# Install ##
# ##############################################################################

include(CMakePackageConfigHelpers)

# 安装头文件
install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})


# 导出所有依赖的目标
install(
  TARGETS
  hardware
  drive
  kinematics
  plog
  gripper
  robot
  trac_ik
  orocos-kdl
  kdl_parser
  ruckig
  nlopt
  EXPORT robot_export-target
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
  PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
  )

# 依赖文件存在位置
set(robot_INSTALL_CONFIGDIR ${CMAKE_INSTALL_LIBDIR}/cmake/robot_export)

# 导出
install(
  EXPORT robot_export-target
  DESTINATION ${robot_INSTALL_CONFIGDIR}
  NAMESPACE rocos::)

# 生成find_package()所需的配置文件
configure_package_config_file(robot-config.cmake.in robot_export-config.cmake
  INSTALL_DESTINATION ${robot_INSTALL_CONFIGDIR})

# 生成find_package()所需的配置文件
write_basic_package_version_file(
  robot_export-config-version.cmake
  VERSION ${rocos_app_VERSION}
  COMPATIBILITY AnyNewerVersion)

# 将生成find_package()所需的配置文件移动到指定位置
install(FILES
  "${CMAKE_CURRENT_BINARY_DIR}/robot_export-config.cmake"
  "${CMAKE_CURRENT_BINARY_DIR}/robot_export-config-version.cmake"
  DESTINATION ${robot_INSTALL_CONFIGDIR})



# [make package]将导出的所有include lib config的压缩包
set(CPACK_GENERATOR "DEB")
set(CPACK_DEBIAN_PACKAGE_MAINTAINER "193355797@qq.com")
set(CPACK_PACKAGE_VERSION ${rocos_app_VERSION})
include(CPack)
