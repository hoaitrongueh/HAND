cmake_minimum_required(VERSION 3.20)

project(HAND VERSION 0.0.1 LANGUAGES CXX)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

add_executable(hand_demo apps/main.cpp src/kinematics.cpp)
target_compile_features(hand_demo PRIVATE cxx_std_17)
set_target_properties(hand_demo PROPERTIES CXX_EXTENSIONS OFF)
target_include_directories(hand_demo PRIVATE "${PROJECT_SOURCE_DIR}/include")

if(MSVC)
    target_compile_options(hand_demo PRIVATE /W4 /permissive-)
else()
    target_compile_options(hand_demo PRIVATE -Wall -Wextra -Wpedantic)
endif()

# Native MuJoCo application (enabled as soon as apps/panda.cpp exists).
# Set MUJOCO_DIR to the unpacked official MuJoCo Windows distribution.
if(EXISTS "${PROJECT_SOURCE_DIR}/apps/panda.cpp")
    set(MUJOCO_DIR "$ENV{MUJOCO_DIR}" CACHE PATH "MuJoCo distribution root")

    if(NOT EXISTS "${MUJOCO_DIR}/include/mujoco/mujoco.h")
        message(FATAL_ERROR "MuJoCo headers not found. Set MUJOCO_DIR to the extracted MuJoCo release root.")
    endif()
    if(NOT EXISTS "${MUJOCO_DIR}/lib/mujoco.lib")
        message(FATAL_ERROR "MuJoCo import library missing: ${MUJOCO_DIR}/lib/mujoco.lib")
    endif()
    if(NOT EXISTS "${MUJOCO_DIR}/bin/mujoco.dll")
        message(FATAL_ERROR "MuJoCo runtime DLL missing: ${MUJOCO_DIR}/bin/mujoco.dll")
    endif()

    add_library(mujoco_native SHARED IMPORTED)
    set_target_properties(mujoco_native PROPERTIES
        IMPORTED_IMPLIB "${MUJOCO_DIR}/lib/mujoco.lib"
        IMPORTED_LOCATION "${MUJOCO_DIR}/bin/mujoco.dll"
        INTERFACE_INCLUDE_DIRECTORIES "${MUJOCO_DIR}/include"
    )

    add_executable(hand_panda apps/panda.cpp)
    target_compile_features(hand_panda PRIVATE cxx_std_17)
    set_target_properties(hand_panda PROPERTIES CXX_EXTENSIONS OFF)
    target_link_libraries(hand_panda PRIVATE mujoco_native)

    if(MSVC)
        target_compile_options(hand_panda PRIVATE /W4 /permissive-)
    else()
        target_compile_options(hand_panda PRIVATE -Wall -Wextra -Wpedantic)
    endif()

    add_custom_command(TARGET hand_panda POST_BUILD
        COMMAND "${CMAKE_COMMAND}" -E copy_if_different
            "${MUJOCO_DIR}/bin/mujoco.dll"
            "$<TARGET_FILE_DIR:hand_panda>"
        COMMENT "Copy MuJoCo runtime DLL beside hand_panda"
    )
endif()
