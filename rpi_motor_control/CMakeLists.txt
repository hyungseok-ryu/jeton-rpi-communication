cmake_minimum_required(VERSION 3.10)
project(motor_control)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)


add_subdirectory(candle)

set(SOURCES
    src/main.cpp
    src/motor.cpp
    src/udp_communication.cpp
    src/uart_communication.cpp
)

add_executable(motor_control ${SOURCES})
target_link_libraries(motor_control candle)
target_include_directories(motor_control PUBLIC ./candle/include ${CMAKE_SOURCE_DIR}/include)
