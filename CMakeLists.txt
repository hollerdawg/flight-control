# Find Zephyr. This also loads Zephyr's build system.
cmake_minimum_required(VERSION 3.13.1)
find_package(Zephyr)
project(sensorfusion VERSION 0.0.1 DESCRIPTION "provides sensor fusion using madgwick algorithm")

# Add your source file to the "app" target. This must come after
# find_package(Zephyr) which defines the target.
target_include_directories(app PRIVATE include)
target_include_directories(app PRIVATE src)
FILE(GLOB app_sources src/*.c)
target_sources(app PRIVATE ${app_sources})
