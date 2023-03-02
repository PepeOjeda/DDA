cmake_minimum_required(VERSION 3.10)
project(DDA)
set(CMAKE_CXX_FLAGS "-w -std=c++11 -O3 -g ${CMAKE_CXX_FLAGS}")

include_directories(
    include
    third_party/glm/glm
)

add_subdirectory(third_party/glm)

add_library(DDA src/DDA.cpp)

target_link_libraries(DDA glm)