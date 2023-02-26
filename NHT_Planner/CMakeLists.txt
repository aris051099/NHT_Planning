cmake_minimum_required (VERSION 3.08)
project(NHT_Planner)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_executable(planner planner.cpp)

add_subdirectory(node)

add_subdirectory(../../public/src ${CMAKE_BINARY_DIR}/public)
add_subdirectory(../../MMLPlayer/ym2612 ${CMAKE_BINARY_DIR}/ym2612 )
add_subdirectory(../../MMLPlayer/mmlplayer ${CMAKE_BINARY_DIR}/mmlplayer )

target_link_libraries(planner node)
target_link_libraries(planner fssimplewindow)