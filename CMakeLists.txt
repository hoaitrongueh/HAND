cmake_minimum_required(VERSION 3.20)

project(HAND VERSION 0.0.1 LANGUAGES CXX)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(HAND_EIGEN_DIR "${CMAKE_CURRENT_SOURCE_DIR}/third_party/eigen")
if(NOT EXISTS "${HAND_EIGEN_DIR}/Eigen/Core")
    message(FATAL_ERROR "Eigen headers missing: ${HAND_EIGEN_DIR}/Eigen/Core")
endif()
add_library(hand_eigen INTERFACE)
add_library(Eigen3::Eigen ALIAS hand_eigen)
target_include_directories(hand_eigen SYSTEM INTERFACE "${HAND_EIGEN_DIR}")
target_compile_features(hand_eigen INTERFACE cxx_std_14)

add_library(hand_core STATIC
    src/kinematics.cpp
    src/drawing.cpp
)

target_compile_features(hand_core PUBLIC cxx_std_17)

set_target_properties(hand_core PROPERTIES
    CXX_EXTENSIONS OFF
)

target_include_directories(hand_core PUBLIC
    "${PROJECT_SOURCE_DIR}/include"
)

target_link_libraries(hand_core PUBLIC
    Eigen3::Eigen
)

add_executable(hand_demo
    apps/main.cpp
)

target_link_libraries(hand_demo PRIVATE
    hand_core
)

option(HAND_BUILD_MUJOCO
    "Build MuJoCo simulation application"
    OFF
)

if(HAND_BUILD_MUJOCO)

    set(MUJOCO_DIR "$ENV{MUJOCO_DIR}"
        CACHE PATH "MuJoCo distribution root"
    )

    if(NOT EXISTS "${MUJOCO_DIR}/include/mujoco/mujoco.h")
        message(FATAL_ERROR
            "MuJoCo headers not found. Set MUJOCO_DIR to the extracted MuJoCo distribution root."
        )
    endif()

    if(NOT EXISTS "${MUJOCO_DIR}/lib/mujoco.lib")
        message(FATAL_ERROR
            "MuJoCo import library missing."
        )
    endif()

    if(NOT EXISTS "${MUJOCO_DIR}/bin/mujoco.dll")
        message(FATAL_ERROR
            "MuJoCo runtime DLL missing."
        )
    endif()

    add_library(mujoco_native SHARED IMPORTED)

    set_target_properties(mujoco_native PROPERTIES
        IMPORTED_IMPLIB "${MUJOCO_DIR}/lib/mujoco.lib"
        IMPORTED_LOCATION "${MUJOCO_DIR}/bin/mujoco.dll"
        INTERFACE_INCLUDE_DIRECTORIES "${MUJOCO_DIR}/include"
    )

    add_library(hand_mujoco STATIC src/mujoco.cpp)
    target_compile_features(hand_mujoco PUBLIC cxx_std_17)
    target_include_directories(hand_mujoco PUBLIC "${PROJECT_SOURCE_DIR}/include")
    target_link_libraries(hand_mujoco PUBLIC mujoco_native)

    add_library(hand_ik STATIC src/ik.cpp)
    target_compile_features(hand_ik PUBLIC cxx_std_17)
    target_include_directories(hand_ik PUBLIC "${PROJECT_SOURCE_DIR}/include")
    target_link_libraries(hand_ik PUBLIC Eigen3::Eigen mujoco_native)

    add_library(hand_execution STATIC src/execution.cpp)
    target_compile_features(hand_execution PUBLIC cxx_std_17)
    target_link_libraries(hand_execution PUBLIC hand_core mujoco_native)
    add_executable(hand_draw apps/draw.cpp)
    target_link_libraries(hand_draw PRIVATE hand_execution)
    add_custom_command(TARGET hand_draw POST_BUILD
        COMMAND "${CMAKE_COMMAND}" -E copy_if_different
            "${MUJOCO_DIR}/bin/mujoco.dll" "$<TARGET_FILE_DIR:hand_draw>"
    )

    add_executable(hand_panda
        apps/panda.cpp
    )

    target_link_libraries(hand_panda PRIVATE
        hand_ik
        hand_mujoco
    )

    add_custom_command(TARGET hand_panda POST_BUILD
        COMMAND "${CMAKE_COMMAND}" -E copy_if_different
            "${MUJOCO_DIR}/bin/mujoco.dll"
            "$<TARGET_FILE_DIR:hand_panda>"
        COMMENT "Copy MuJoCo runtime DLL"
    )

endif()

include(CTest)

set(HAND_TARGETS hand_core hand_demo)

if(BUILD_TESTING)
    add_executable(test_kinematics tests/test_kinematics.cpp)
    target_link_libraries(test_kinematics PRIVATE hand_core)
    add_test(NAME kinematics COMMAND test_kinematics)
    list(APPEND HAND_TARGETS test_kinematics)
    add_executable(test_drawing tests/test_drawing.cpp)
    target_link_libraries(test_drawing PRIVATE hand_core)
    add_test(NAME drawing_contract COMMAND test_drawing)
    list(APPEND HAND_TARGETS test_drawing)
    if(TARGET hand_ik)
        add_executable(test_panda_ik tests/test_panda_ik.cpp)
        target_link_libraries(test_panda_ik PRIVATE hand_ik)
        target_compile_definitions(test_panda_ik PRIVATE
            HAND_PANDA_MODEL="${PROJECT_SOURCE_DIR}/simulation/models/franka_emika_panda/scene.xml"
            HAND_PANDA_SOURCE="${PROJECT_SOURCE_DIR}/simulation/models/franka_emika_panda/panda.xml"
            HAND_IK_OUTPUT="${PROJECT_BINARY_DIR}/ik_validation"
        )
        add_custom_command(TARGET test_panda_ik POST_BUILD
            COMMAND "${CMAKE_COMMAND}" -E copy_if_different
                "${MUJOCO_DIR}/bin/mujoco.dll" "$<TARGET_FILE_DIR:test_panda_ik>"
        )
        set(PANDA_IK_CASES baseline directions tolerance displacements configurations
            near_limits jacobian indexing atomic_rejection disabled_limits
            accepted_monotonicity line_search_rejection tiny_improvement invalid_options numerical_validity
            invalid_model missing_joint missing_joint_at_target unreachable
            constrained_target singular last_update nonfinite_state invalid_state_limits
            reachable_joint_limit nan_x nan_y nan_z inf_x inf_y inf_z
            neginf_x neginf_y neginf_z stress)
        foreach(case IN LISTS PANDA_IK_CASES)
            add_test(NAME panda_ik.${case} COMMAND test_panda_ik ${case})
            set_tests_properties(panda_ik.${case} PROPERTIES TIMEOUT 60
                WORKING_DIRECTORY "${PROJECT_BINARY_DIR}")
        endforeach()
        foreach(case missing_joint_at_target last_update nonfinite_state invalid_state_limits
                reachable_joint_limit nan_x nan_y nan_z inf_x inf_y inf_z
                neginf_x neginf_y neginf_z)
            set_tests_properties(panda_ik.${case} PROPERTIES LABELS "known_defect")
        endforeach()
        list(APPEND HAND_TARGETS test_panda_ik)
    endif()
endif()

if(TARGET hand_panda)
    list(APPEND HAND_TARGETS hand_mujoco hand_ik hand_panda hand_execution hand_draw)
endif()

foreach(target IN LISTS HAND_TARGETS)
    set_target_properties(${target} PROPERTIES CXX_EXTENSIONS OFF)
    if(MSVC)
        target_compile_options(${target} PRIVATE
            /W4 /permissive-
        )
    else()
        target_compile_options(${target} PRIVATE
            -Wall -Wextra -Wpedantic
        )
    endif()
endforeach()
