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
)

srp_enable_warnings(srp_core)

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
)

srp_enable_warnings(srp_window)

if(SRP_BUILD_TESTS)
    enable_testing()

    add_executable(srp_core_tests
        tests/unit/core/version_test.cpp
    )

    target_link_libraries(srp_core_tests
        PRIVATE
            srp_core
    )

    srp_enable_warnings(srp_core_tests)

    add_test(
        NAME core.version
        COMMAND srp_core_tests
    )

    add_executable(srp_core_loop_tests
        tests/unit/core/fixed_step_loop_test.cpp
    )

    target_link_libraries(srp_core_loop_tests
        PRIVATE
            srp_core
    )

    srp_enable_warnings(srp_core_loop_tests)

    add_test(
        NAME core.fixed_step_loop
        COMMAND srp_core_loop_tests
    )

    add_executable(srp_core_config_tests
        tests/unit/core/app_config_test.cpp
    )

    target_link_libraries(srp_core_config_tests
        PRIVATE
            srp_core
    )

    srp_enable_warnings(srp_core_config_tests)

    add_test(
        NAME core.app_config
        COMMAND srp_core_config_tests
    )
endif()
