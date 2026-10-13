cmake_minimum_required(VERSION 3.20)

project(HAND VERSION 0.0.1 LANGUAGES CXX)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Add the robotics library from src/ when its first module is implemented.
add_executable(hand_demo apps/main.cpp)
target_compile_features(hand_demo PRIVATE cxx_std_17)
set_target_properties(hand_demo PROPERTIES CXX_EXTENSIONS OFF)
target_include_directories(hand_demo PRIVATE "${PROJECT_SOURCE_DIR}/include")

if(MSVC)
    target_compile_options(hand_demo PRIVATE /W4 /permissive-)
else()
    target_compile_options(hand_demo PRIVATE -Wall -Wextra -Wpedantic)
endif()
