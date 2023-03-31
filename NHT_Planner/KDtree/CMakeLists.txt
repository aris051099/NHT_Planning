set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)


add_library(KDtree KDtree.cpp KDtree.h)

target_include_directories(KDtree PUBLIC ../KDtree)