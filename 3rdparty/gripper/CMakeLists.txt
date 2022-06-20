cmake_minimum_required (VERSION 3.5)

project(gripper)


add_library(gripper INTERFACE)
target_include_directories(gripper
    INTERFACE
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}/${PROJECT_NAME}>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/gripper_client_cpp>
)
add_library(JC_helper::gripper ALIAS gripper)