cmake_minimum_required(VERSION 3.7.2)
project(kdl_parser)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 设置CMake模块路径
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_LIST_DIR}/cmake)

# 找到所需的依赖项，不查找ROS 2版本的urdf
find_package(Eigen3 REQUIRED)
find_package(urdfdom REQUIRED) # 仅查找非ROS版本
find_package(TinyXML REQUIRED)
find_package(TinyXML2 REQUIRED)

# 添加包含路径
include_directories(include ${TinyXML_INCLUDE_DIRS} ${TinyXML2_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR})

# 添加库和可执行文件
add_library(${PROJECT_NAME} SHARED src/kdl_parser.cpp)
target_link_libraries(${PROJECT_NAME} PUBLIC ${TinyXML_LIBRARIES} ${TinyXML2_LIBRARIES} ${urdfdom_LIBRARIES} ${EIGEN3_LIBRARIES} orocos-kdl)
target_include_directories(${PROJECT_NAME} PUBLIC ${PROJECT_SOURCE_DIR}/include)

add_executable(test_kdl_parser src/test_kdl_parser.cpp)
target_link_libraries(test_kdl_parser PRIVATE ${PROJECT_NAME})

# 安装指令
include(GNUInstallDirs)
install(DIRECTORY include/${PROJECT_NAME} DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
install(TARGETS ${PROJECT_NAME} test_kdl_parser
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION bin)
