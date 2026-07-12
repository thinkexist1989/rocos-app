cmake_minimum_required(VERSION 3.15)

add_library(sol2 INTERFACE)
target_include_directories(sol2 INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(sol2 INTERFACE lua::lua)

add_library(sol2::sol2 ALIAS sol2)
