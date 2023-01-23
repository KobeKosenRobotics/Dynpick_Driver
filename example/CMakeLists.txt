cmake_minimum_required(VERSION 3.4)
# project name 
project(dynpick_driver_ex)


# Library
#add_library(libgpio /home/WiringPi/wiringPi/wiringPi.c /home/WiringPi/wiringPi/wiringPi.h)
include_directories(../lib/include)
link_directories(../lib/build)


add_executable(dynpick_driver_ex example.cpp)
target_link_libraries(dynpick_driver_ex dynpick_driver)