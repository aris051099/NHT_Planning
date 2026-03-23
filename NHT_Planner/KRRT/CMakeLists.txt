
add_library(KRRT KRRT.cpp KRRT.h)
target_link_libraries(KRRT node map object KDtree)
target_include_directories(KRRT PUBLIC ../KRRT)