cmake_minimum_required (VERSION 3.08)
project(NHT_Planner)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_subdirectory(Xstate)
add_subdirectory(node)
add_subdirectory(map)
add_subdirectory(object)
add_subdirectory(KDtree)
add_subdirectory(KRRT)
add_subdirectory(../../public/src ${CMAKE_BINARY_DIR}/public)
add_subdirectory(../../MMLPlayer/ym2612 ${CMAKE_BINARY_DIR}/ym2612 )
add_subdirectory(../../MMLPlayer/mmlplayer ${CMAKE_BINARY_DIR}/mmlplayer )

add_executable(planner planner.cpp)


target_link_libraries(planner fssimplewindow)
target_link_libraries(planner map)
target_link_libraries(planner ysbitmapfont)
target_link_libraries(planner object)
target_link_libraries(planner Xstate)
target_link_libraries(planner KDtree)
target_link_libraries(planner node)
target_link_libraries(planner KRRT)
