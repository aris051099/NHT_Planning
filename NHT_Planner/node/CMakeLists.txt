
add_library(node node.cpp node.h)

target_link_libraries(node Xstate)

target_include_directories(node PUBLIC ../node)
