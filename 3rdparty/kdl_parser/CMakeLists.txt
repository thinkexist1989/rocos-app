cmake_minimum_required(VERSION 3.7.2)

project(kdl_parser)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
# set(CMAKE_INSTALL_PREFIX  /home/abc418/123/kdl_parser/output)
# SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/output/bin)  
find_package(catkin QUIET
  COMPONENTS rosconsole cmake_modules
)

if(NOT catkin_FOUND)
  # use local copies of FindTinyXML.cmake and FindTinyXML2.cmake from
  # 'cmake_modules' (https://github.com/ros/cmake_modules)
  list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_LIST_DIR}/cmake)
endif()

find_package(Eigen3 QUIET)
find_package(urdfdom REQUIRED)
find_package(TinyXML REQUIRED)
find_package(TinyXML2 REQUIRED)

# check for rosconsole
# We check additionally for catkin to distinguish between an "official" ROS distribution
# and the one provided in the distribution's repository.
find_package(rosconsole QUIET)
if(rosconsole_FOUND AND catkin_FOUND)
  add_definitions(-DHAS_ROS)
endif()

find_package(urdf QUIET)
if(urdf_FOUND)
  add_definitions(-DHAS_URDF)
  include_directories(${urdf_INCLUDE_DIRS})
endif()

include_directories(include ${TinyXML_INCLUDE_DIRS} ${TinyXML2_INCLUDE_DIRS} ${urdfdom_INCLUDE_DIRS}   ${EIGEN3_INCLUDE_DIR} )

if(catkin_FOUND)
  link_directories(${catkin_LIBRARY_DIRS})
  include_directories(${catkin_INCLUDE_DIRS})

  catkin_package(
    LIBRARIES ${PROJECT_NAME} ${orocos_kdl_LIBRARIES}
    INCLUDE_DIRS include
    CATKIN_DEPENDS rosconsole urdf
    DEPENDS orocos_kdl TinyXML TinyXML2
  )
endif()

add_library(${PROJECT_NAME} SHARED src/kdl_parser.cpp)
target_link_libraries(${PROJECT_NAME} PUBLIC ${TinyXML_LIBRARIES} ${TinyXML2_LIBRARIES} 
)
target_link_libraries(${PROJECT_NAME}  PUBLIC ${urdfdom_LIBRARIES}   ${EIGEN3_LIBRARIES}   orocos-kdl )
target_include_directories( ${PROJECT_NAME}  PUBLIC  ${PROJECT_SOURCE_DIR}/include)

if(catkin_FOUND)
  target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
endif()

if(urdf_FOUND)
  target_link_libraries(${PROJECT_NAME} ${urdf_LIBRARIES})
endif()

if(WIN32)
  target_compile_definitions(${PROJECT_NAME} PRIVATE "KDL_PARSER_BUILDING_DLL")
endif()

add_executable(check_kdl_parser src/check_kdl_parser.cpp )
target_link_libraries(check_kdl_parser  PRIVATE ${PROJECT_NAME})

if(catkin_FOUND AND CATKIN_ENABLE_TESTING)
  find_package(catkin REQUIRED COMPONENTS roscpp rostest)
  add_rostest_gtest(test_kdl_parser test/test_kdl_parser.launch test/test_kdl_parser.cpp )
  target_link_libraries(test_kdl_parser ${PROJECT_NAME})

  add_rostest_gtest(test_inertia_rpy test/test_inertia_rpy.launch test/test_inertia_rpy.cpp )
  target_link_libraries(test_inertia_rpy ${PROJECT_NAME})
endif()

if(catkin_FOUND)
  # How does CATKIN do this?
  #rosbuild_add_rostest(${PROJECT_SOURCE_DIR}/test/test_kdl_parser.launch)
  
  # Install library
  install(TARGETS ${PROJECT_NAME}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    )

  install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
else()
  install(TARGETS ${PROJECT_NAME}  check_kdl_parser
          LIBRARY DESTINATION lib
          ARCHIVE DESTINATION lib
          RUNTIME DESTINATION bin
          )
  install(DIRECTORY include/${PROJECT_NAME}/
          DESTINATION include)

endif()
