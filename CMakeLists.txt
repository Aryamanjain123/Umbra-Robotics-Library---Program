cmake_minimum_required(VERSION 3.19)
project(vexlib_host CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Library sources (host build excludes PROS-only content at compile time with __has_include guards)
file(GLOB_RECURSE VEXLIB_HEADERS "include/*.hpp")
file(GLOB_RECURSE VEXLIB_SOURCES "src/*.cpp")

add_library(vexlib ${VEXLIB_HEADERS} ${VEXLIB_SOURCES})
target_include_directories(vexlib PUBLIC include)
target_compile_definitions(vexlib PUBLIC VEXLIB_HOST=1)
if(MSVC)
  target_compile_options(vexlib PRIVATE /W4)
else()
  target_compile_options(vexlib PRIVATE -Wall -Wextra -Wpedantic)
endif()

# Tests (FetchContent for GTest)
include(FetchContent)
set(BUILD_GMOCK OFF CACHE BOOL "" FORCE)
set(INSTALL_GTEST OFF CACHE BOOL "" FORCE)
FetchContent_Declare(
  googletest
  URL https://github.com/google/googletest/archive/refs/tags/v1.15.2.zip
)
FetchContent_MakeAvailable(googletest)

enable_testing()
add_executable(test_pid tests/test_pid.cpp)
target_link_libraries(test_pid PRIVATE vexlib GTest::gtest_main)
include(GoogleTest)
gtest_discover_tests(test_pid)
