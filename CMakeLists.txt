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
find_package(Boost REQUIRED COMPONENTS date_time filesystem system chrono thread)
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

set(DEP_LIBS
        dl
        rt
        Threads::Threads
        Boost::boost
        Boost::date_time
        Boost::filesystem
        Boost::system
        Boost::chrono
        Boost::thread
        yaml-cpp
)

add_library(hardware src/command.cpp
        src/status_word.cpp
        src/control_word.cpp
        src/drive_state.cpp
        src/hardware.cpp
        src/hardware_interface.cpp
        src/hardware_sim.cpp
)
target_link_libraries(hardware
        PRIVATE
        ${DEP_LIBS})
add_library(rocos::hardware ALIAS hardware)

add_executable(${PROJECT_NAME} src/rocosAppMain.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}
        PRIVATE
        Qt5::Core
        rocos::hardware
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
       rocos::hardware
       )
add_test(NAME unit_test COMMAND unit_test)
