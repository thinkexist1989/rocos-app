cmake_minimum_required(VERSION 3.5)
project(trac_ik_lib)


find_package(Eigen3 QUIET)
if(NOT EIGEN3_FOUND)
  include(${PROJ_SOURCE_DIR}/cmake/FindEigen3.cmake)
endif()


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
  

install(DIRECTORY 
  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
)

install(TARGETS trac_ik
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
