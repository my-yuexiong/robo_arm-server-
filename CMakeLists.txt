cmake_minimum_required(VERSION 3.10)

project(main)

aux_source_directory(. SRC_SUB)

add_executable(main ${SRC_SUB})

#add_executable(unlock ${SRC_SUB})
find_package(Threads REQUIRED)
#fine_library(WIRINGPI_LIB WiringPi)

target_link_libraries(main wiringPi)
target_link_libraries(main Threads::Threads)
include_directories(Inc)
