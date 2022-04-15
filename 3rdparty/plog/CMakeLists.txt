cmake_minimum_required(VERSION 3.0)

option(PLOG_BUILD_SAMPLES "Build plog's samples." OFF)

project(plog LANGUAGES CXX)


# Make sure install paths work on all platforms.
if(NOT CMAKE_INSTALL_INCLUDEDIR)
    include(GNUInstallDirs)
endif()

add_library(plog INTERFACE)
target_include_directories(plog
    INTERFACE
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}/${PROJECT_NAME}>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

if(ANDROID)
    target_link_libraries(plog INTERFACE log)
endif()

add_library(plog::plog ALIAS plog)

#making sure we can build standalone under windows
get_filename_component(CURRENT_SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}   ABSOLUTE)
get_filename_component(SOURCE_DIR ${CMAKE_SOURCE_DIR} ABSOLUTE)



install(
    DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/include/plog
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
    FILES_MATCHING # headers only
    PATTERN "*.h"
)
