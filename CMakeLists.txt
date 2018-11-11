project(virtualmouse)

set(CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS} " -O2 -std=c++11 -Wall -Wextra")

add_executable(virtualmouse main.cpp)

target_link_libraries(virtualmouse pthread)