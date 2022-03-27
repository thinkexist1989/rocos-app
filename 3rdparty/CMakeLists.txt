cmake_minimum_required (VERSION 3.5)

project(kdl-trac_ik-kdl_parser)
set(EXECUTABLE_OUTPUT_PATH  ${PROJECT_SOURCE_DIR}/bin)

add_subdirectory(kdl)
add_subdirectory(nlopt)
add_subdirectory(trac_ik)
add_subdirectory(kdl_parser)
#add_subdirectory(my_trac_ik)
