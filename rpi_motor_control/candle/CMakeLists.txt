cmake_minimum_required(VERSION 3.5)
set(CMAKE_CXX_STANDARD 14)

project(candle_lib)

# change the version and tag here
set(CANDLE_LIB_VERSION 3.5.3)

set(VERSION_TAG 'r')

set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(CMAKE_BUILD_TYPE Release)
add_compile_options(-Wall -Wextra -Wpedantic)

if(ARCH STREQUAL "armhf")
    message("[CANDLE] Compiling for armhf")
    set(CMAKE_CXX_COMPILER /usr/bin/arm-linux-gnueabihf-g++)
    set(CMAKE_C_COMPILER /usr/bin/arm-linux-gnueabihf-gcc)
elseif(ARCH STREQUAL "aarm64")
    message("[CANDLE] Compiling for aarm64")
    set(CMAKE_CXX_COMPILER /usr/bin/aarch64-linux-gnu-g++)
    set(CMAKE_C_COMPILER /usr/bin/aarch64-linux-gnu-gcc)
endif()

file(GLOB CANDLE_SOURCE "src/*cpp")

if(CANDLE_BUILD_STATIC)
    set(CANDLE_BUILD_STATIC "TRUE")
    set(CANDLE_BUILD_SHARED "FALSE")
    add_library(candle STATIC ${CANDLE_SOURCE})
else()
    set(CANDLE_BUILD_STATIC "FALSE")
    set(CANDLE_BUILD_SHARED "TRUE")
    add_library(candle SHARED ${CANDLE_SOURCE})
endif()

message(STATUS "\nCANDLE_BUILD_SHARED: ${CANDLE_BUILD_SHARED}\nCANDLE_BUILD_STATIC: ${CANDLE_BUILD_STATIC}")

target_include_directories(candle PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(candle pthread -static-libgcc -static-libstdc++)
set_target_properties(candle PROPERTIES PUBLIC_HEADER candle.hpp)

string(REGEX MATCH "([0-9]+)\\.([0-9]+)\\.([0-9]+)" _ "${CANDLE_LIB_VERSION}")
target_compile_definitions(candle
    PRIVATE CANDLE_VMAJOR=${CMAKE_MATCH_1}
    PRIVATE CANDLE_VMINOR=${CMAKE_MATCH_2}
    PRIVATE CANDLE_VREVISION=${CMAKE_MATCH_3}
    PRIVATE CANDLE_VTAG=${VERSION_TAG}
)

if(MAKE_TESTS)
    enable_testing()
    add_subdirectory(test)
    set(CANDLE_OMIT_EXAMPLES ON)
endif()
