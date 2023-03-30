set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)


add_library(object object.cpp object.h)

target_link_libraries(object fssimplewindow ysbitmapfont)


target_include_directories(object PUBLIC ../object)