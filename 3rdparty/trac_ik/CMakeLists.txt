cmake_minimum_required(VERSION 3.5)
project(trac_ik_lib)

include(GNUInstallDirs)

set(${PROJECT_NAME}_INSTALL_CONFIGDIR ${CMAKE_INSTALL_PREFIX}/config)
set(${PROJECT_NAME}_INSTALL_CMAKEDIR ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME})

find_package(Eigen3 QUIET)
if (NOT EIGEN3_FOUND)
    include(${PROJ_SOURCE_DIR}/cmake/FindEigen3.cmake)
endif ()


find_package(Boost REQUIRED COMPONENTS date_time system)


add_library(trac_ik SHARED
        src/kdl_tl.cpp
        src/nlopt_ik.cpp
        src/trac_ik.cpp
        )


target_link_libraries(trac_ik
        PUBLIC
        ${Boost_LIBRARIES}
        ${EIGEN3_LIBRARIES}
        nlopt
        orocos-kdl
        kdl_parser
        )

target_include_directories(trac_ik
        PUBLIC ${PROJECT_SOURCE_DIR}/include
        PRIVATE ${EIGEN3_INCLUDE_DIR}
        )

# Add support for installation
include(CMakePackageConfigHelpers)

## Install headers
install(DIRECTORY include/trac_ik DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

install(DIRECTORY
        DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
        )

install(TARGETS trac_ik
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

#install(TARGETS trac_ik
#        EXPORT trac_ik-targets
#        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}  # 动态库安装路径
#        ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}  # 静态库安装路径
#        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}  # 可执行文件安装路径
#        PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}  # 头文件安装路径
#        )
#
#install(EXPORT trac_ik-targets
#        FILE trac_ik-targets.cmake
#        NAMESPACE trac_ik::
#        DESTINATION ${${PROJECT_NAME}_INSTALL_CMAKEDIR})