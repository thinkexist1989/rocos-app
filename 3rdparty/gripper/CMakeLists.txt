cmake_minimum_required (VERSION 3.5)

project(gripper)


add_library(gripper INTERFACE)
target_include_directories(gripper
    INTERFACE
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}/${PROJECT_NAME}>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/gripper_client_cpp>
)
add_library(JC_helper::gripper ALIAS gripper)

# Add support for installation
include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

set(${PROJECT_NAME}_INSTALL_CONFIGDIR ${CMAKE_INSTALL_PREFIX}/config)
set(${PROJECT_NAME}_INSTALL_CMAKEDIR ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME})

install(FILES gripper_client_cpp/gripper.hpp
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

install(TARGETS gripper
        EXPORT gripper-targets
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}  # 动态库安装路径
        ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}  # 静态库安装路径
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}  # 可执行文件安装路径
        PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}  # 头文件安装路径
        )

install(EXPORT gripper-targets
        FILE gripper-targets.cmake
        NAMESPACE gripper::
        DESTINATION ${${PROJECT_NAME}_INSTALL_CMAKEDIR})