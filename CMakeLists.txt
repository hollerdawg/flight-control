# CMakeLists.txt for Zephyr Quadcopter Application
cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(quadcopter_flight_controller)

target_sources(app PRIVATE src/main.c src/mahony.c src/pid_control.c)
target_sources_ifdef(CONFIG_FLIGHT_CTL_RC_IBUS app PRIVATE src/ibus.c)
