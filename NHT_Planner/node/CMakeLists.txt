set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)


add_library(node node.cpp dynamics.h state_template.h node.h)

target_include_directories(node PUBLIC ../node)
