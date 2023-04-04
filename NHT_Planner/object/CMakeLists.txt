
add_library(object object.cpp object.h)

target_link_libraries(object fssimplewindow ysbitmapfont)


target_include_directories(object PUBLIC ../object)