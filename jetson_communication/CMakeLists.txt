cmake_minimum_required(VERSION 3.10)
project(communication)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(SOURCES
    src/main.cpp
    src/motor.cpp
    src/udp_communication.cpp
    src/uart_communication.cpp
)

add_executable(main ${SOURCES})
target_include_directories(main PUBLIC ${CMAKE_SOURCE_DIR}/include)
