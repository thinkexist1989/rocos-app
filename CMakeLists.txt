#[[
Copyright 2021, Yang Luo"
This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.

@Author
Yang Luo, PHD
Shenyang Institute of Automation, Chinese Academy of Sciences.
email: luoyang@sia.cn

@Created on: 2021.12.06
]]

cmake_minimum_required(VERSION 3.10)
project(rocos_app)

# add_compile_options(-std=c++11)
set(CMAKE_CXX_STANDARD 11)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

################
## Dependencies ##
################
# find_package(Boost REQUIRED COMPONENTS system)
find_package(Boost REQUIRED COMPONENTS date_time filesystem system chrono)
find_package(yaml-cpp REQUIRED)
find_package(Threads REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Core)


###########
## Build ##
###########

include_directories(
  include
  include/${PROJECT_NAME}
  include/sdk
)

link_directories(
  lib
)

file(GLOB_RECURSE SRCS src/*.cpp)
file(GLOB_RECURSE HDRS include/*.h include/*.hpp)

add_executable(${PROJECT_NAME} ${SRCS})

set(DEP_LIBS
        Threads::Threads
        Boost::boost
        Boost::date_time
        Boost::filesystem
        Boost::system
        Boost::chrono
        yaml-cpp
        Qt5::Core
)

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}
        PRIVATE
        ${DEP_LIBS}
)

#############
## Install ##
#############



#############
## Testing ##
#############

enable_testing()
add_executable(unit_test test/unit_test.cpp)
target_link_libraries(unit_test
       PRIVATE
       ${DEP_LIBS}
       )
add_test(NAME unit_test COMMAND unit_test)
