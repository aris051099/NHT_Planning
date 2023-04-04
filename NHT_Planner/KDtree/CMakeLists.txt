
add_library(KDtree KDtree.cpp KDtree.h)
target_link_libraries(KDtree Xstate)
target_include_directories(KDtree PUBLIC ../KDtree)