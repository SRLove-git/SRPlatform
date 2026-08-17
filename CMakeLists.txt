cmake_minimum_required(VERSION 3.24)

project(SRPlatform VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
include(CompilerWarnings)

include(FetchContent)

set(SPDLOG_BUILD_EXAMPLE OFF CACHE BOOL "" FORCE)
set(SPDLOG_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(SPDLOG_INSTALL OFF CACHE BOOL "" FORCE)

FetchContent_Declare(
    spdlog
    GIT_REPOSITORY https://github.com/gabime/spdlog.git
    GIT_TAG v1.15.3
)

FetchContent_MakeAvailable(spdlog)

set(JSON_BuildTests OFF CACHE BOOL "" FORCE)

FetchContent_Declare(
    json
    GIT_REPOSITORY https://github.com/nlohmann/json.git
    GIT_TAG v3.11.3
)

FetchContent_MakeAvailable(json)

FetchContent_Declare(
    glm
    GIT_REPOSITORY https://github.com/g-truc/glm.git
    GIT_TAG 1.0.1
)

FetchContent_MakeAvailable(glm)

set(INSTALL_GTEST OFF CACHE BOOL "" FORCE)
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)

FetchContent_Declare(
    googletest
    GIT_REPOSITORY https://github.com/google/googletest.git
    GIT_TAG v1.15.2
)

FetchContent_MakeAvailable(googletest)

option(SRP_BUILD_TESTS "Build SRPlatform unit tests" ON)

add_library(srp_core
    src/core/version.cpp
    src/core/loop/fixed_step_loop.cpp
    src/core/logging.cpp
    src/core/config/app_config.cpp
)

target_include_directories(srp_core
    PUBLIC
        "${CMAKE_CURRENT_SOURCE_DIR}/src"
)

target_link_libraries(srp_core
    PUBLIC
        spdlog::spdlog
        nlohmann_json::nlohmann_json
        glm::glm
)

srp_enable_warnings(srp_core)

add_library(srp_physics
    src/physics/rigid_body.cpp
    src/physics/physics_world.cpp
)

target_include_directories(srp_physics
    PUBLIC
        "${CMAKE_CURRENT_SOURCE_DIR}/src"
)

target_link_libraries(srp_physics
    PUBLIC
        srp_core
)

srp_enable_warnings(srp_physics)

add_executable(srp_cli
    src/app/main.cpp
)

target_link_libraries(srp_cli
    PRIVATE
        srp_core
)

srp_enable_warnings(srp_cli)

add_executable(srp_window WIN32
    src/app/window_main.cpp
)

target_link_libraries(srp_window
    PRIVATE
        srp_core
        user32
        gdi32
        opengl32
        glu32
)

srp_enable_warnings(srp_window)

if(SRP_BUILD_TESTS)
    enable_testing()

    add_executable(srp_unit_tests
        tests/unit/core/version_test.cpp
        tests/unit/core/fixed_step_loop_test.cpp
        tests/unit/core/app_config_test.cpp
        tests/unit/core/math_types_test.cpp
        tests/unit/physics/rigid_body_test.cpp
        tests/unit/physics/physics_world_test.cpp
    )

    target_link_libraries(srp_unit_tests
        PRIVATE
            srp_core
            srp_physics
            GTest::gtest_main
    )

    srp_enable_warnings(srp_unit_tests)

    add_test(
        NAME core.unit
        COMMAND srp_unit_tests
    )
endif()
