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

set(LUA_BUILD_COMPILER OFF CACHE BOOL "" FORCE)
set(LUA_BUILD_INTERPRETER OFF CACHE BOOL "" FORCE)

FetchContent_Declare(
    lua
    GIT_REPOSITORY https://github.com/jonpalmisc/cmake_lua.git
    GIT_TAG fb8ca8ff834102f0a8751b8978aeee0e1d04a874
)

FetchContent_MakeAvailable(lua)

set(SOL2_TESTS OFF CACHE BOOL "" FORCE)
set(SOL2_EXAMPLES OFF CACHE BOOL "" FORCE)
set(SOL2_ENABLE_INSTALL OFF CACHE BOOL "" FORCE)

FetchContent_Declare(
    sol2
    GIT_REPOSITORY https://github.com/ThePhD/sol2.git
    GIT_TAG c1f95a773c6f8f4fde8ca3efe872e7286afe4444
)

FetchContent_MakeAvailable(sol2)

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
    src/physics/collision_shape.cpp
    src/physics/aabb.cpp
    src/physics/broadphase.cpp
    src/physics/narrowphase.cpp
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

add_library(srp_circuit
    src/circuit/circuit_model.cpp
    src/circuit/linear_solver.cpp
    src/circuit/dc_solver.cpp
    src/circuit/transient_solver.cpp
    src/circuit/digital_solver.cpp
    src/circuit/netlist_loader.cpp
)

target_include_directories(srp_circuit
    PUBLIC
        "${CMAKE_CURRENT_SOURCE_DIR}/src"
)

target_link_libraries(srp_circuit
    PUBLIC
        nlohmann_json::nlohmann_json
)

srp_enable_warnings(srp_circuit)

add_library(srp_mod
    src/mod/mod_manifest.cpp
    src/mod/gltf_model.cpp
)

target_include_directories(srp_mod
    PUBLIC
        "${CMAKE_CURRENT_SOURCE_DIR}/src"
)

target_link_libraries(srp_mod
    PUBLIC
        glm::glm
        nlohmann_json::nlohmann_json
)

srp_enable_warnings(srp_mod)

add_library(srp_control
    src/control/pid_controller.cpp
    src/control/hover_controller.cpp
)

target_include_directories(srp_control
    PUBLIC
        "${CMAKE_CURRENT_SOURCE_DIR}/src"
)

srp_enable_warnings(srp_control)

add_library(srp_bridge
    src/bridge/battery_model.cpp
    src/bridge/bridge.cpp
    src/bridge/car_entity.cpp
    src/bridge/car_control_bus.cpp
    src/bridge/dc_motor_model.cpp
    src/bridge/distance_sensor_model.cpp
    src/bridge/drone_control_bus.cpp
    src/bridge/drone_entity.cpp
    src/bridge/encoder_model.cpp
    src/bridge/imu_model.cpp
    src/bridge/propeller_model.cpp
    src/bridge/quadcopter_model.cpp
    src/bridge/state_recorder.cpp
)

target_include_directories(srp_bridge
    PUBLIC
        "${CMAKE_CURRENT_SOURCE_DIR}/src"
)

target_link_libraries(srp_bridge
    PUBLIC
        srp_core
        srp_physics
)

srp_enable_warnings(srp_bridge)

add_library(srp_scripting
    src/scripting/car_closed_loop_demo.cpp
    src/scripting/drone_closed_loop_demo.cpp
    src/scripting/lua_script_host.cpp
)

target_include_directories(srp_scripting
    PUBLIC
        "${CMAKE_CURRENT_SOURCE_DIR}/src"
)

target_link_libraries(srp_scripting
    PUBLIC
        srp_core
        srp_bridge
        lua::lang
        sol2::sol2
)

srp_enable_warnings(srp_scripting)

add_library(srp_rendering
    src/rendering/debug_draw.cpp
)

target_include_directories(srp_rendering
    PUBLIC
        "${CMAKE_CURRENT_SOURCE_DIR}/src"
)

target_link_libraries(srp_rendering
    PUBLIC
        srp_core
        srp_physics
        opengl32
        glu32
    PRIVATE
        srp_mod
)

srp_enable_warnings(srp_rendering)

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
        srp_rendering
        srp_scripting
        user32
        gdi32
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
        tests/unit/physics/collision_shape_test.cpp
        tests/unit/physics/broadphase_test.cpp
        tests/unit/physics/narrowphase_test.cpp
        tests/unit/physics/contact_test.cpp
        tests/unit/physics/contact_solver_test.cpp
        tests/unit/physics/restitution_friction_test.cpp
        tests/unit/physics/joint_test.cpp
        tests/unit/physics/wheel_constraint_test.cpp
        tests/unit/physics/regression_test.cpp
        tests/unit/circuit/circuit_model_test.cpp
        tests/unit/circuit/dc_solver_test.cpp
        tests/unit/circuit/transient_solver_test.cpp
        tests/unit/circuit/source_test.cpp
        tests/unit/circuit/nonlinear_test.cpp
        tests/unit/circuit/digital_solver_test.cpp
        tests/unit/circuit/regression_test.cpp
        tests/unit/circuit/netlist_loader_test.cpp
        tests/unit/control/pid_controller_test.cpp
        tests/unit/control/hover_controller_test.cpp
        tests/unit/mod/mod_manifest_test.cpp
        tests/unit/mod/gltf_model_test.cpp
        tests/unit/bridge/battery_model_test.cpp
        tests/unit/bridge/bridge_test.cpp
        tests/unit/bridge/car_entity_test.cpp
        tests/unit/bridge/dc_motor_model_test.cpp
        tests/unit/bridge/distance_sensor_model_test.cpp
        tests/unit/bridge/drone_entity_test.cpp
        tests/unit/bridge/encoder_model_test.cpp
        tests/unit/bridge/imu_model_test.cpp
        tests/unit/bridge/propeller_model_test.cpp
        tests/unit/bridge/quadcopter_model_test.cpp
        tests/unit/bridge/state_recorder_test.cpp
        tests/unit/scripting/lua_script_host_test.cpp
        tests/unit/scripting/car_closed_loop_demo_test.cpp
        tests/unit/scripting/drone_closed_loop_demo_test.cpp
        tests/unit/scripting/script_control_test.cpp
    )

    target_link_libraries(srp_unit_tests
        PRIVATE
            srp_core
            srp_physics
            srp_circuit
            srp_control
            srp_mod
            srp_bridge
            srp_scripting
            GTest::gtest_main
    )

    srp_enable_warnings(srp_unit_tests)

    add_test(
        NAME core.unit
        COMMAND srp_unit_tests
    )
endif()
