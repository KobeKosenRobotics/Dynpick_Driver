cmake_minimum_required(VERSION 3.4)

include_directories("include")
add_library(dynpick_driver SHARED src/dynpick.cpp)
