set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)


add_library(map map.cpp map.h)

target_link_libraries(map fssimplewindow)
target_include_directories(map PUBLIC ../map)
