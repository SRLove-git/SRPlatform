cmake_minimum_required(VERSION 3.24)

project(SRPlatform VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
include(CompilerWarnings)

option(SRP_BUILD_TESTS "Build SRPlatform unit tests" ON)

add_library(srp_core
    src/core/version.cpp
)

target_include_directories(srp_core
    PUBLIC
        "${CMAKE_CURRENT_SOURCE_DIR}/src"
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
endif()
