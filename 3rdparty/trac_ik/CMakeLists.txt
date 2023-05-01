cmake_minimum_required(VERSION 3.5)
project(trac_ik_lib)

find_package(Eigen3 QUIET)
if(NOT EIGEN3_FOUND)
  include(${PROJ_SOURCE_DIR}/cmake/FindEigen3.cmake)
endif()


find_package(Boost REQUIRED COMPONENTS date_time system)

add_library(trac_ik SHARED src/kdl_tl.cpp src/nlopt_ik.cpp src/trac_ik.cpp)

target_link_libraries(trac_ik PUBLIC ${Boost_LIBRARIES} ${EIGEN3_LIBRARIES}
                                     nlopt orocos-kdl kdl_parser)

# target_include_directories(
# trac_ik
# PUBLIC ${PROJECT_SOURCE_DIR}/include
# PRIVATE ${EIGEN3_INCLUDE_DIR})
target_include_directories(
  trac_ik
  PUBLIC $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  PRIVATE $<BUILD_INTERFACE:${EIGEN3_INCLUDE_DIR}>
  $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)

install(DIRECTORY include/trac_ik
DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})


install(
  TARGETS trac_ik
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
  )
