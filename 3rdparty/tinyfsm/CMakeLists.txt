cmake_minimum_required (VERSION 3.5)

project(FSM)


add_library(FSM INTERFACE)
target_include_directories(FSM
    INTERFACE
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}/${PROJECT_NAME}>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

# Add support for installation
include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

set(${PROJECT_NAME}_INSTALL_CONFIGDIR ${CMAKE_INSTALL_PREFIX}/config)
set(${PROJECT_NAME}_INSTALL_CMAKEDIR ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME})

install(FILES include/tinyfsm.hpp
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

install(TARGETS FSM
        EXPORT FSM-targets
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}  # 动态库安装路径
        ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}  # 静态库安装路径
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}  # 可执行文件安装路径
        PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}  # 头文件安装路径
        )

install(EXPORT FSM-targets
        FILE FSM-targets.cmake
        NAMESPACE FSM::
        DESTINATION ${${PROJECT_NAME}_INSTALL_CMAKEDIR})
