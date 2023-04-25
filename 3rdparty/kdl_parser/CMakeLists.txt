cmake_minimum_required(VERSION 3.7.2)

project(kdl_parser)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(catkin_FOUND OFF)





list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_LIST_DIR}/cmake)
find_package(urdfdom REQUIRED)
find_package(TinyXML REQUIRED)
find_package(TinyXML2 REQUIRED)




include_directories(include ${TinyXML_INCLUDE_DIRS} ${TinyXML2_INCLUDE_DIRS} ${urdfdom_INCLUDE_DIRS}   ${EIGEN3_INCLUDE_DIR} )



add_library(${PROJECT_NAME} SHARED src/kdl_parser.cpp)
target_link_libraries(${PROJECT_NAME} PUBLIC ${TinyXML_LIBRARIES} ${TinyXML2_LIBRARIES} 
)
target_link_libraries(${PROJECT_NAME}  PUBLIC ${urdfdom_LIBRARIES}   ${EIGEN3_LIBRARIES}   orocos-kdl )
# target_include_directories( ${PROJECT_NAME}  PUBLIC  ${PROJECT_SOURCE_DIR}/include)
target_include_directories(
  ${PROJECT_NAME} PUBLIC $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
                 $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)


# if(urdf_FOUND)
#   target_link_libraries(${PROJECT_NAME}  PUBLIC ${urdf_LIBRARIES})
# endif()

if(WIN32)
  target_compile_definitions(${PROJECT_NAME} PRIVATE "KDL_PARSER_BUILDING_DLL")
endif()

# add_executable(test_kdl_parser src/test_kdl_parser.cpp )
# target_link_libraries(test_kdl_parser  PRIVATE ${PROJECT_NAME})




  # install(TARGETS ${PROJECT_NAME}  test_kdl_parser
          # LIBRARY DESTINATION lib
          # ARCHIVE DESTINATION lib
          # RUNTIME DESTINATION bin
          # )
  install(DIRECTORY include/${PROJECT_NAME}
          DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})


