# CMakeLists.txt for Math Unit Tests
cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(test_math)

target_sources(app PRIVATE
    src/test_math.c
    ../../src/mahony.c
    ../../src/pid_control.c
)

target_include_directories(app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../../src)
