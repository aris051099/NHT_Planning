add_executable(planner planner.cpp)

target_link_libraries(planner node)
target_link_libraries(planner fssimplewindow)