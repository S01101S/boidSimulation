cmake_minimum_required(VERSION 3.22)
project(boidSimulation)

set(CMAKE_CXX_STANDARD 20)

find_package(SFML 2.5 COMPONENTS graphics window system REQUIRED)

add_executable(boidSimulation main.cpp)

target_link_libraries(boidSimulation PRIVATE sfml-graphics sfml-window sfml-system)