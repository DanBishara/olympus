cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(olympus)

target_include_directories(app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/common)
add_subdirectory(components/ringbuffer)
add_subdirectory(components/accelerometer)
add_subdirectory(components/LED)
add_subdirectory(components/PPG)
add_subdirectory(components/BLE)
add_subdirectory(components/boost)
add_subdirectory(components/heartrate)
target_sources(app PRIVATE src/main.cpp)
