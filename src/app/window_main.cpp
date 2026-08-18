#define WIN32_LEAN_AND_MEAN
#include "core/config/app_config.hpp"
#include "core/loop/fixed_step_loop.hpp"
#include "core/logging.hpp"
#include "bridge/distance_sensor_model.hpp"
#include "editor/course_catalog.hpp"
#include "editor/circuit_editor.hpp"
#include "editor/playback_controller.hpp"
#include "editor/scene_editor.hpp"
#include "editor/script_editor.hpp"
#include "editor/sim_observer.hpp"
#include "editor/simulation_control.hpp"
#include "editor/simulation_recorder.hpp"
#include "editor/time_series.hpp"
#include "rendering/debug_draw.hpp"
#include "scripting/car_closed_loop_demo.hpp"
#include "scripting/arm_closed_loop_demo.hpp"
#include "scripting/drone_closed_loop_demo.hpp"
#include "scripting/lua_script_host.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <fstream>
#include <optional>
#include <sstream>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>
#include <windows.h>
#include <windowsx.h>

#include <imgui.h>
#include <imgui_impl_opengl2.h>
#include <imgui_impl_win32.h>

#include <GL/gl.h>
#include <GL/glu.h>

// Forward declaration copied from the ImGui Win32 backend: the backend header
// intentionally keeps this declaration behind '#if 0' to avoid dragging
// <windows.h> dependencies into other translation units.
extern IMGUI_IMPL_API LRESULT ImGui_ImplWin32_WndProcHandler(
    HWND hWnd,
    UINT msg,
    WPARAM wParam,
    LPARAM lParam);

namespace
{

constexpr wchar_t kWindowClassName[] = L"SRPlatformWindow";

std::uint64_t g_simulation_ticks = 0;
std::uint64_t g_render_frames = 0;

double g_camera_yaw = 0.0;
double g_camera_pitch = 0.35;
double g_camera_distance = 6.0;
bool g_camera_orbiting = false;
int g_last_mouse_x = 0;
int g_last_mouse_y = 0;

bool g_move_mode = false;
bool g_move_dragging = false;

srp::editor::SceneEditor* g_scene_editor = nullptr;
srp::editor::CircuitEditor* g_circuit_editor = nullptr;
srp::editor::ScriptEditor* g_script_editor = nullptr;
srp::editor::CourseCatalog g_course_catalog;
srp::editor::CourseCatalog* g_course_catalog_ptr = nullptr;
std::optional<srp::circuit::ComponentType> g_circuit_placement;
std::string g_script_status;

srp::editor::SimObserver g_observer;
srp::bridge::DistanceSensorModel g_front_sensor;
srp::bridge::DistanceSensorModel g_left_sensor;
srp::bridge::DistanceSensorModel g_right_sensor;
bool g_observation_enabled = true;
srp::editor::SimulationControl g_sim_control;
srp::editor::SimulationRecorder g_recorder;
srp::editor::PlaybackController g_playback;
bool g_playback_active = false;
srp::core::FixedStepLoop::Clock::time_point g_last_frame_time{};
bool g_has_last_frame_time = false;

struct ActiveDemo
{
    std::unique_ptr<srp::scripting::CarClosedLoopDemo> car;
    std::unique_ptr<srp::scripting::DroneClosedLoopDemo> drone;
    std::unique_ptr<srp::scripting::ArmClosedLoopDemo> arm;
    std::string kind;

    bool has() const
    {
        return !kind.empty();
    }

    srp::scripting::LuaScriptHost& host()
    {
        if (kind == "drone")
        {
            return drone->host();
        }
        if (kind == "arm")
        {
            return arm->host();
        }
        return car->host();
    }

    bool loadScript(const std::string& source, std::string& error)
    {
        if (!host().load("controller", source))
        {
            error = host().lastError().value_or("unknown script error");
            return false;
        }
        return true;
    }

    bool step(double dt)
    {
        if (kind == "drone")
        {
            return drone->step(dt);
        }
        if (kind == "arm")
        {
            return arm->step(dt);
        }
        return car->step(dt);
    }

    double elapsedTime() const
    {
        if (kind == "drone")
        {
            return drone->drone().elapsedTime();
        }
        if (kind == "arm")
        {
            return 0.0;
        }
        return car->car().elapsedTime();
    }
};

ActiveDemo g_demo;

std::vector<srp::editor::BodySnapshot> captureBodySnapshots(
    const srp::editor::SceneEditor& scene_editor)
{
    std::vector<srp::editor::BodySnapshot> snapshots;

    const srp::physics::RigidBodyState* ground =
        scene_editor.world().body(scene_editor.groundBody());
    if (ground != nullptr)
    {
        srp::editor::BodySnapshot snapshot;
        snapshot.name = "ground";
        snapshot.kind = "plane";
        snapshot.position = ground->position;
        snapshot.orientation = ground->orientation;
        snapshots.push_back(std::move(snapshot));
    }

    for (const srp::editor::BodyEntry& entry : scene_editor.entries())
    {
        const srp::physics::RigidBodyState* body =
            scene_editor.world().body(entry.id);
        if (body == nullptr)
        {
            continue;
        }
        srp::editor::BodySnapshot snapshot;
        snapshot.name = entry.name;
        snapshot.kind = srp::editor::shapeKindName(entry.kind);
        snapshot.position = body->position;
        snapshot.orientation = body->orientation;
        snapshots.push_back(std::move(snapshot));
    }

    if (g_demo.kind == "car" && g_demo.car != nullptr)
    {
        const srp::physics::RigidBodyState* chassis =
            g_demo.car->car().chassisBody();
        if (chassis != nullptr)
        {
            srp::editor::BodySnapshot snapshot;
            snapshot.name = "car_chassis";
            snapshot.kind = "chassis";
            snapshot.position = chassis->position;
            snapshot.orientation = chassis->orientation;
            snapshots.push_back(std::move(snapshot));
        }

        const srp::physics::RigidBodyState* wheel =
            g_demo.car->car().wheelBody();
        if (wheel != nullptr)
        {
            srp::editor::BodySnapshot snapshot;
            snapshot.name = "car_wheel";
            snapshot.kind = "wheel";
            snapshot.position = wheel->position;
            snapshot.orientation = wheel->orientation;
            snapshots.push_back(std::move(snapshot));
        }
    }
    else if (g_demo.kind == "drone" && g_demo.drone != nullptr)
    {
        const srp::physics::RigidBodyState* body = g_demo.drone->drone().body();
        if (body != nullptr)
        {
            srp::editor::BodySnapshot snapshot;
            snapshot.name = "drone_body";
            snapshot.kind = "drone";
            snapshot.position = body->position;
            snapshot.orientation = body->orientation;
            snapshots.push_back(std::move(snapshot));
        }
    }
    else if (g_demo.kind == "arm" && g_demo.arm != nullptr)
    {
        for (std::size_t i = 0; i < g_demo.arm->arm().jointCount(); ++i)
        {
            const auto [position, orientation] =
                g_demo.arm->arm().linkTransform(i);
            srp::editor::BodySnapshot snapshot;
            snapshot.name = "arm_link_" + std::to_string(i);
            snapshot.kind = "arm_link";
            snapshot.position = position;
            snapshot.orientation = orientation;
            snapshots.push_back(std::move(snapshot));
        }
    }

    return snapshots;
}

void drawPlaybackBodies(const std::vector<srp::editor::BodySnapshot>& bodies)
{
    for (const srp::editor::BodySnapshot& snapshot : bodies)
    {
        if (snapshot.kind == "box")
        {
            glColor3d(0.35, 0.75, 1.0);
            srp::physics::BoxShape shape;
            shape.half_extents = srp::math::Vec3(0.5);
            srp::rendering::drawCollisionShape(
                shape,
                snapshot.position,
                snapshot.orientation);
        }
        else if (snapshot.kind == "sphere")
        {
            glColor3d(0.35, 0.75, 1.0);
            srp::physics::SphereShape shape;
            shape.radius = 0.5;
            srp::rendering::drawCollisionShape(
                shape,
                snapshot.position,
                snapshot.orientation);
        }
        else if (snapshot.kind == "cylinder")
        {
            glColor3d(0.35, 0.75, 1.0);
            srp::physics::CylinderShape shape;
            shape.half_height = 0.5;
            shape.radius = 0.3;
            srp::rendering::drawCollisionShape(
                shape,
                snapshot.position,
                snapshot.orientation);
        }
        else if (snapshot.kind == "chassis")
        {
            glColor3d(0.2, 0.9, 0.4);
            srp::physics::BoxShape shape;
            shape.half_extents = srp::math::Vec3(0.2, 0.1, 0.2);
            srp::rendering::drawCollisionShape(
                shape,
                snapshot.position,
                snapshot.orientation);
        }
        else if (snapshot.kind == "wheel")
        {
            glColor3d(0.95, 0.7, 0.2);
            srp::physics::SphereShape shape;
            shape.radius = 0.3;
            srp::rendering::drawCollisionShape(
                shape,
                snapshot.position,
                snapshot.orientation);
        }
        else if (snapshot.kind == "drone")
        {
            glColor3d(0.8, 0.4, 0.9);
            srp::physics::SphereShape shape;
            shape.radius = 0.25;
            srp::rendering::drawCollisionShape(
                shape,
                snapshot.position,
                snapshot.orientation);
        }
        else if (snapshot.kind == "arm_link")
        {
            glColor3d(0.9, 0.5, 0.2);
            srp::physics::BoxShape shape;
            shape.half_extents = srp::math::Vec3(0.5, 0.07, 0.07);
            srp::rendering::drawCollisionShape(
                shape,
                snapshot.position,
                snapshot.orientation);
        }
        else
        {
            glColor3d(0.45, 0.45, 0.5);
            srp::physics::PlaneShape shape;
            srp::rendering::drawCollisionShape(
                shape,
                snapshot.position,
                snapshot.orientation);
        }
    }
}

constexpr float kCircuitScale = 60.0f;
constexpr double kSideBeamAngle = 0.45;

const char* shortComponentName(srp::circuit::ComponentType type)
{
    switch (type)
    {
    case srp::circuit::ComponentType::kResistor:
        return "R";
    case srp::circuit::ComponentType::kCapacitor:
        return "C";
    case srp::circuit::ComponentType::kInductor:
        return "L";
    case srp::circuit::ComponentType::kVoltageSource:
        return "V";
    case srp::circuit::ComponentType::kCurrentSource:
        return "I";
    case srp::circuit::ComponentType::kDiode:
        return "D";
    case srp::circuit::ComponentType::kSwitch:
        return "SW";
    case srp::circuit::ComponentType::kDigitalSource:
        return "DIG";
    case srp::circuit::ComponentType::kLogicGate:
        return "GATE";
    case srp::circuit::ComponentType::kDFlipFlop:
        return "DFF";
    case srp::circuit::ComponentType::kPwmSource:
        return "PWM";
    }
    return "?";
}

ImVec2 circuitToScreen(const srp::math::Vec2& logical, const ImVec2& origin)
{
    return ImVec2(
        origin.x + static_cast<float>(logical.x) * kCircuitScale,
        origin.y + static_cast<float>(logical.y) * kCircuitScale);
}

srp::math::Vec2 screenToCircuit(const ImVec2& screen, const ImVec2& origin)
{
    return srp::math::Vec2(
        (screen.x - origin.x) / kCircuitScale,
        (screen.y - origin.y) / kCircuitScale);
}

constexpr double kCameraTargetY = 0.5;

srp::math::Vec3 cameraEye()
{
    return srp::math::Vec3(
        g_camera_distance * std::cos(g_camera_pitch) * std::sin(g_camera_yaw),
        kCameraTargetY + g_camera_distance * std::sin(g_camera_pitch),
        g_camera_distance * std::cos(g_camera_pitch) * std::cos(g_camera_yaw));
}

// Builds a world-space ray through a client-space pixel.
std::pair<srp::math::Vec3, srp::math::Vec3> screenRay(
    int x,
    int y,
    int width,
    int height)
{
    constexpr srp::math::Vec3 kUp(0.0, 1.0, 0.0);
    constexpr srp::math::Vec3 kTarget(0.0, kCameraTargetY, 0.0);

    const srp::math::Vec3 eye = cameraEye();
    const srp::math::Vec3 forward = glm::normalize(kTarget - eye);
    const srp::math::Vec3 right = glm::normalize(glm::cross(forward, kUp));
    const srp::math::Vec3 up = glm::cross(right, forward);

    const double aspect = static_cast<double>(width) / static_cast<double>(height);
    const double half_fov = srp::math::radians(45.0) * 0.5;
    const double ndc_x =
        (2.0 * static_cast<double>(x) / static_cast<double>(width) - 1.0) *
        std::tan(half_fov) * aspect;
    const double ndc_y =
        (1.0 - 2.0 * static_cast<double>(y) / static_cast<double>(height)) *
        std::tan(half_fov);

    const srp::math::Vec3 point =
        eye + forward + right * ndc_x + up * ndc_y;
    return {eye, glm::normalize(point - eye)};
}

void handleViewportClick(HWND window, int x, int y)
{
    if (g_scene_editor == nullptr)
    {
        return;
    }

    RECT client_rect{};
    GetClientRect(window, &client_rect);
    const int width = client_rect.right - client_rect.left;
    const int height = client_rect.bottom - client_rect.top;
    if (width <= 0 || height <= 0)
    {
        return;
    }

    const auto [origin, direction] = screenRay(x, y, width, height);
    const std::optional<srp::physics::BodyId> selected = g_scene_editor->selected();

    if (g_move_mode && selected.has_value())
    {
        const std::optional<srp::math::Vec3> target =
            srp::editor::SceneEditor::pickGround(origin, direction);
        if (target.has_value())
        {
            g_scene_editor->moveTo(*selected, *target);
            g_move_dragging = true;
            SetCapture(window);
        }
        return;
    }

    const std::optional<srp::physics::BodyId> hit =
        srp::editor::SceneEditor::pick(
            g_scene_editor->world(),
            origin,
            direction);
    if (hit.has_value())
    {
        g_scene_editor->select(*hit);
    }
    else
    {
        g_scene_editor->deselect();
    }
}

void handleViewportDrag(HWND window, int x, int y)
{
    if (g_scene_editor == nullptr || !g_move_dragging)
    {
        return;
    }

    const std::optional<srp::physics::BodyId> selected = g_scene_editor->selected();
    if (!selected.has_value())
    {
        g_move_dragging = false;
        return;
    }

    RECT client_rect{};
    GetClientRect(window, &client_rect);
    const int width = client_rect.right - client_rect.left;
    const int height = client_rect.bottom - client_rect.top;
    if (width <= 0 || height <= 0)
    {
        return;
    }

    const auto [origin, direction] = screenRay(x, y, width, height);
    const std::optional<srp::math::Vec3> target =
        srp::editor::SceneEditor::pickGround(origin, direction);
    if (target.has_value())
    {
        g_scene_editor->moveTo(*selected, *target);
    }
}

void renderScene(
    int width,
    int height,
    const srp::editor::SceneEditor& scene_editor)
{
    if (width <= 0 || height <= 0)
    {
        return;
    }

    glViewport(0, 0, width, height);
    glClearColor(0.10f, 0.11f, 0.15f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    glEnable(GL_DEPTH_TEST);

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(
        45.0,
        static_cast<double>(width) / static_cast<double>(height),
        0.1,
        100.0);

    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    constexpr srp::math::Vec3 kTarget(0.0, kCameraTargetY, 0.0);
    const srp::math::Vec3 eye = cameraEye();
    gluLookAt(
        eye.x, eye.y, eye.z,
        kTarget.x, kTarget.y, kTarget.z,
        0.0, 1.0, 0.0);

    if (g_playback_active)
    {
        const std::optional<std::vector<srp::editor::BodySnapshot>> frame =
            g_playback.frame();
        if (frame.has_value())
        {
            drawPlaybackBodies(*frame);
        }
        return;
    }

    const srp::physics::PhysicsWorld& world = scene_editor.world();
    const std::optional<srp::physics::BodyId> selected = scene_editor.selected();

    for (const srp::physics::BodyId id : world.bodyIds())
    {
        const srp::physics::RigidBodyState* body = world.body(id);
        const srp::physics::CollisionShape* shape = world.shape(id);
        if (body == nullptr || shape == nullptr)
        {
            continue;
        }

        const bool is_ground = std::holds_alternative<srp::physics::PlaneShape>(*shape);
        const bool is_selected = selected.has_value() && *selected == id;
        if (is_selected)
        {
            glColor3d(1.0, 0.85, 0.1);
        }
        else if (is_ground)
        {
            glColor3d(0.45, 0.45, 0.5);
        }
        else
        {
            glColor3d(0.35, 0.75, 1.0);
        }
        srp::rendering::drawCollisionShape(
            *shape,
            body->position,
            body->orientation);
    }

    if (g_demo.kind == "car" && g_demo.car != nullptr)
    {
        const srp::physics::RigidBodyState* chassis =
            g_demo.car->car().chassisBody();
        if (chassis != nullptr)
        {
            glColor3d(0.2, 0.9, 0.4);
            srp::rendering::drawCollisionShape(
                g_demo.car->car().chassisShape(),
                chassis->position,
                chassis->orientation);
        }

        const srp::physics::RigidBodyState* wheel =
            g_demo.car->car().wheelBody();
        if (wheel != nullptr)
        {
            glColor3d(0.95, 0.7, 0.2);
            srp::rendering::drawCollisionShape(
                g_demo.car->car().wheelShape(),
                wheel->position,
                wheel->orientation);
        }
    }
    else if (g_demo.kind == "drone" && g_demo.drone != nullptr)
    {
        const srp::physics::RigidBodyState* body = g_demo.drone->drone().body();
        if (body != nullptr)
        {
            glColor3d(0.8, 0.4, 0.9);
            srp::rendering::drawCollisionShape(
                g_demo.drone->drone().bodyShape(),
                body->position,
                body->orientation);
        }
    }
    else if (g_demo.kind == "arm" && g_demo.arm != nullptr)
    {
        for (std::size_t i = 0; i < g_demo.arm->arm().jointCount(); ++i)
        {
            const auto [position, orientation] =
                g_demo.arm->arm().linkTransform(i);
            glColor3d(0.9, 0.5, 0.2);
            srp::physics::BoxShape shape;
            shape.half_extents = srp::math::Vec3(0.5, 0.07, 0.07);
            srp::rendering::drawCollisionShape(
                shape,
                position,
                orientation);
        }
        glColor3d(0.95, 0.85, 0.2);
        const srp::math::Vec3 end = g_demo.arm->arm().endEffectorPosition();
        srp::physics::SphereShape effector;
        effector.radius = 0.06;
        srp::rendering::drawCollisionShape(
            effector,
            end,
            srp::math::Quat(1.0, 0.0, 0.0, 0.0));
    }

    glColor3d(1.0, 0.2, 0.2);
    for (const srp::physics::Contact& contact : world.contacts())
    {
        srp::rendering::drawContactPoint(contact.point);
    }
}

void drawCircuitPanel()
{
    if (g_circuit_editor == nullptr)
    {
        return;
    }

    ImGui::SetNextWindowPos(ImVec2(10.0f, 440.0f), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize(ImVec2(520.0f, 320.0f), ImGuiCond_FirstUseEver);
    ImGui::Begin("Circuit");

    constexpr srp::circuit::ComponentType kPalette[] = {
        srp::circuit::ComponentType::kResistor,
        srp::circuit::ComponentType::kCapacitor,
        srp::circuit::ComponentType::kInductor,
        srp::circuit::ComponentType::kVoltageSource,
        srp::circuit::ComponentType::kCurrentSource,
        srp::circuit::ComponentType::kDiode,
        srp::circuit::ComponentType::kSwitch,
        srp::circuit::ComponentType::kPwmSource,
        srp::circuit::ComponentType::kDigitalSource,
        srp::circuit::ComponentType::kLogicGate,
        srp::circuit::ComponentType::kDFlipFlop};

    for (const srp::circuit::ComponentType type : kPalette)
    {
        const bool armed = g_circuit_placement.has_value() && *g_circuit_placement == type;
        if (ImGui::Button(shortComponentName(type)))
        {
            g_circuit_placement = type;
        }
        if (armed)
        {
            ImGui::SameLine();
            ImGui::Text("<- click canvas to place");
        }
        else
        {
            ImGui::SameLine();
        }
    }

    if (ImGui::Button("Export"))
    {
        std::ofstream stream("assets/circuits/editor_netlist.json");
        stream << g_circuit_editor->toNetlistJson().dump(2);
        srp::core::logInfo("exported circuit to assets/circuits/editor_netlist.json");
    }
    ImGui::SameLine();
    if (ImGui::Button("Load"))
    {
        std::ifstream stream("assets/circuits/editor_netlist.json");
        if (!stream.is_open())
        {
            srp::core::logError("no saved circuit to load");
        }
        else
        {
            nlohmann::json json;
            stream >> json;
            std::string error;
            if (!g_circuit_editor->loadNetlistJson(json, error))
            {
                srp::core::logError("failed to load circuit: " + error);
            }
            else
            {
                srp::core::logInfo("loaded circuit from assets/circuits/editor_netlist.json");
            }
        }
    }
    ImGui::SameLine();
    ImGui::TextDisabled("Components: %zu", g_circuit_editor->circuit().components().size());

    const ImVec2 canvas_size = ImGui::GetContentRegionAvail();
    ImGui::InvisibleButton("##circuit_canvas", canvas_size);
    const bool canvas_clicked = ImGui::IsItemClicked(ImGuiMouseButton_Left);
    const bool canvas_right_clicked = ImGui::IsItemClicked(ImGuiMouseButton_Right);
    const ImVec2 canvas_min = ImGui::GetItemRectMin();
    const ImVec2 canvas_max = ImGui::GetItemRectMax();
    const ImVec2 canvas_origin = ImVec2(
        (canvas_min.x + canvas_max.x) * 0.5f,
        (canvas_min.y + canvas_max.y) * 0.5f);

    ImDrawList* draw_list = ImGui::GetWindowDrawList();
    draw_list->AddRectFilled(
        canvas_min,
        canvas_max,
        IM_COL32(18, 20, 26, 255));

    // Wires: every node connects its ports.
    std::unordered_map<srp::circuit::NodeId, std::vector<srp::circuit::PortId>> node_ports;
    for (const srp::circuit::Port& port : g_circuit_editor->circuit().ports())
    {
        if (port.node != srp::circuit::kInvalidNodeId)
        {
            node_ports[port.node].push_back(port.id);
        }
    }
    for (const auto& [node_id, ports] : node_ports)
    {
        if (ports.size() < 2)
        {
            continue;
        }
        const ImVec2 first = circuitToScreen(
            g_circuit_editor->portPosition(ports.front()),
            canvas_origin);
        for (std::size_t i = 1; i < ports.size(); ++i)
        {
            const ImVec2 next = circuitToScreen(
                g_circuit_editor->portPosition(ports[i]),
                canvas_origin);
            draw_list->AddLine(first, next, IM_COL32(90, 170, 220, 255), 2.0f);
        }
        if (node_id == srp::circuit::kGroundNodeId)
        {
            draw_list->AddRectFilled(
                ImVec2(first.x - 6.0f, first.y + 4.0f),
                ImVec2(first.x + 6.0f, first.y + 8.0f),
                IM_COL32(120, 130, 150, 255));
        }
    }

    // Pending wire preview.
    const std::optional<srp::circuit::PortId> pending = g_circuit_editor->pendingWirePort();
    if (pending.has_value())
    {
        const ImVec2 pending_screen = circuitToScreen(
            g_circuit_editor->portPosition(*pending),
            canvas_origin);
        draw_list->AddLine(
            pending_screen,
            ImGui::GetIO().MousePos,
            IM_COL32(90, 230, 120, 220),
            2.0f);
    }

    for (const srp::circuit::Component& component : g_circuit_editor->circuit().components())
    {
        const srp::math::Vec2 position = g_circuit_editor->componentPosition(component.id);
        const ImVec2 screen = circuitToScreen(position, canvas_origin);
        const bool is_selected = g_circuit_editor->selected() == component.id;
        const ImU32 body_color = IM_COL32(48, 52, 66, 255);
        const ImU32 border_color = is_selected
            ? IM_COL32(240, 210, 60, 255)
            : IM_COL32(110, 120, 140, 255);
        draw_list->AddRectFilled(
            ImVec2(screen.x - 50.0f, screen.y - 24.0f),
            ImVec2(screen.x + 50.0f, screen.y + 24.0f),
            body_color);
        draw_list->AddRect(
            ImVec2(screen.x - 50.0f, screen.y - 24.0f),
            ImVec2(screen.x + 50.0f, screen.y + 24.0f),
            border_color,
            4.0f);

        const char* label = shortComponentName(component.definition.type);
        const ImVec2 label_size = ImGui::CalcTextSize(label);
        draw_list->AddText(
            ImVec2(screen.x - label_size.x * 0.5f, screen.y - label_size.y * 0.5f),
            IM_COL32(220, 225, 235, 255),
            label);

        for (const srp::circuit::PortId port_id : component.ports)
        {
            const srp::circuit::Port* port = g_circuit_editor->circuit().port(port_id);
            const ImVec2 port_screen = circuitToScreen(
                g_circuit_editor->portPosition(port_id),
                canvas_origin);
            const bool is_pending = pending.has_value() && *pending == port_id;
            const ImU32 port_color = port != nullptr && port->node != srp::circuit::kInvalidNodeId
                ? IM_COL32(90, 200, 120, 255)
                : IM_COL32(200, 90, 90, 255);
            draw_list->AddCircleFilled(port_screen, 4.0f, port_color);
            if (is_pending)
            {
                draw_list->AddCircle(port_screen, 7.0f, IM_COL32(90, 230, 120, 255), 0, 2.0f);
            }
        }
    }

    if (canvas_clicked)
    {
        const ImVec2 mouse = ImGui::GetIO().MousePos;
        const srp::math::Vec2 logical = screenToCircuit(mouse, canvas_origin);

        if (g_circuit_placement.has_value())
        {
            const srp::circuit::ComponentId id = g_circuit_editor->addComponent(
                *g_circuit_placement,
                logical);
            if (id != srp::circuit::kInvalidComponentId)
            {
                g_circuit_editor->select(id);
            }
            g_circuit_placement.reset();
        }
        else
        {
            srp::circuit::PortId hit_port = srp::circuit::kInvalidPortId;
            float best_distance = 12.0f;
            for (const srp::circuit::Port& port : g_circuit_editor->circuit().ports())
            {
                const ImVec2 port_screen = circuitToScreen(
                    g_circuit_editor->portPosition(port.id),
                    canvas_origin);
                const float distance = std::hypot(
                    mouse.x - port_screen.x,
                    mouse.y - port_screen.y);
                if (distance < best_distance)
                {
                    best_distance = distance;
                    hit_port = port.id;
                }
            }

            if (hit_port != srp::circuit::kInvalidPortId)
            {
                if (pending.has_value() && *pending != hit_port)
                {
                    g_circuit_editor->wire(*pending, hit_port);
                }
                else
                {
                    g_circuit_editor->setPendingWirePort(hit_port);
                }
            }
            else
            {
                srp::circuit::ComponentId hit_component = srp::circuit::kInvalidComponentId;
                for (const srp::circuit::Component& component :
                     g_circuit_editor->circuit().components())
                {
                    const srp::math::Vec2 distance =
                        g_circuit_editor->componentPosition(component.id) - logical;
                    if (std::abs(distance.x) <= 1.0 && std::abs(distance.y) <= 0.7)
                    {
                        hit_component = component.id;
                        break;
                    }
                }

                if (hit_component != srp::circuit::kInvalidComponentId)
                {
                    g_circuit_editor->select(hit_component);
                }
                else
                {
                    g_circuit_editor->deselect();
                    g_circuit_editor->cancelWire();
                }
            }
        }
    }

    if (canvas_right_clicked)
    {
        g_circuit_placement.reset();
        g_circuit_editor->cancelWire();
    }

    if (ImGui::IsWindowFocused() && ImGui::IsKeyPressed(ImGuiKey_Delete))
    {
        const std::optional<srp::circuit::ComponentId> selected = g_circuit_editor->selected();
        if (selected.has_value())
        {
            g_circuit_editor->removeComponent(*selected);
        }
    }

    if (pending.has_value())
    {
        ImGui::Text(
            "Wiring: port %u - click another port to connect (right click cancels)",
            *pending);
    }
    else if (g_circuit_placement.has_value())
    {
        ImGui::Text(
            "Placing %s - click the canvas (right click cancels)",
            shortComponentName(*g_circuit_placement));
    }

    const std::optional<srp::circuit::ComponentId> selected = g_circuit_editor->selected();
    if (selected.has_value())
    {
        const srp::circuit::Component* component =
            g_circuit_editor->circuit().component(*selected);
        if (component != nullptr)
        {
            ImGui::Text("Selected: %s", component->definition.name.c_str());
            for (const srp::circuit::PortId port_id : component->ports)
            {
                const srp::circuit::Port* port = g_circuit_editor->circuit().port(port_id);
                if (port != nullptr)
                {
                    const std::string node_name =
                        port->node != srp::circuit::kInvalidNodeId &&
                                g_circuit_editor->circuit().node(port->node) != nullptr
                            ? g_circuit_editor->circuit().node(port->node)->name
                            : "floating";
                    ImGui::Text("  %s -> %s", port->name.c_str(), node_name.c_str());
                }
            }
        }
    }

    ImGui::End();
}

void drawSeriesPlot(
    ImDrawList* draw_list,
    const char* label,
    const srp::editor::TimeSeries& series,
    const ImVec2& origin,
    const ImVec2& size)
{
    const ImU32 background = IM_COL32(24, 26, 34, 255);
    const ImU32 border = IM_COL32(90, 100, 120, 255);
    draw_list->AddRectFilled(origin, ImVec2(origin.x + size.x, origin.y + size.y), background);
    draw_list->AddRect(origin, ImVec2(origin.x + size.x, origin.y + size.y), border);

    const ImU32 text_color = IM_COL32(190, 200, 215, 255);
    draw_list->AddText(ImVec2(origin.x + 4.0f, origin.y + 2.0f), text_color, label);

    if (series.size() < 2)
    {
        draw_list->AddText(
            ImVec2(origin.x + 4.0f, origin.y + size.y - 16.0f),
            IM_COL32(120, 130, 150, 255),
            "no data");
        return;
    }

    const double min_value = series.minimum().value_or(0.0);
    const double max_value = series.maximum().value_or(0.0);
    double range = max_value - min_value;
    if (range < 1e-9)
    {
        range = 1.0;
    }

    const std::vector<std::pair<double, double>>& samples = series.samples();
    const std::size_t count = samples.size();
    std::vector<ImVec2> points;
    points.reserve(count);
    for (std::size_t i = 0; i < count; ++i)
    {
        const float x = origin.x + 2.0f +
            static_cast<float>(i) / static_cast<float>(count - 1) * (size.x - 4.0f);
        const float normalized = static_cast<float>(
            (samples[i].second - min_value) / range);
        const float y = origin.y + size.y - 14.0f -
            normalized * (size.y - 20.0f);
        points.emplace_back(x, y);
    }
    draw_list->AddPolyline(
        points.data(),
        static_cast<int>(points.size()),
        IM_COL32(90, 190, 250, 255),
        ImDrawFlags_None,
        1.6f);

    char range_label[128]{};
    std::snprintf(
        range_label,
        sizeof(range_label),
        "min=%.3f max=%.3f last=%.3f",
        min_value,
        max_value,
        series.latestValue().value_or(0.0));
    draw_list->AddText(
        ImVec2(origin.x + 4.0f, origin.y + size.y - 14.0f),
        text_color,
        range_label);
}

void drawObservationPanel()
{
    ImGui::SetNextWindowPos(ImVec2(560.0f, 10.0f), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize(ImVec2(330.0f, 680.0f), ImGuiCond_FirstUseEver);
    ImGui::Begin("Observations");

    ImGui::Checkbox("Record observations", &g_observation_enabled);
    if (ImGui::Button("Clear"))
    {
        g_observer.clear();
    }
    ImGui::SameLine();
    ImGui::TextDisabled("samples: %zu", g_observer.channelNames().empty()
        ? 0u
        : g_observer.channel(g_observer.channelNames().front()).size());

    static bool show_voltage = true;
    static bool show_current = true;
    static bool show_speed = true;
    static bool show_sensor = true;
    ImGui::Checkbox("battery voltage", &show_voltage);
    ImGui::Checkbox("motor current", &show_current);
    ImGui::Checkbox("chassis speed", &show_speed);
    ImGui::Checkbox("front distance", &show_sensor);
    ImGui::Separator();

    const float plot_width = ImGui::GetContentRegionAvail().x;
    ImDrawList* draw_list = ImGui::GetWindowDrawList();
    const ImVec2 plot_size(plot_width, 78.0f);
    ImVec2 origin = ImGui::GetCursorScreenPos();

    if (show_voltage)
    {
        drawSeriesPlot(
            draw_list,
            "battery_voltage_v",
            g_observer.channel("battery_voltage_v"),
            origin,
            plot_size);
        origin.y += plot_size.y + 6.0f;
    }
    if (show_current)
    {
        drawSeriesPlot(
            draw_list,
            "motor_current_a",
            g_observer.channel("motor_current_a"),
            origin,
            plot_size);
        origin.y += plot_size.y + 6.0f;
    }
    if (show_speed)
    {
        drawSeriesPlot(
            draw_list,
            "chassis_speed_m_s",
            g_observer.channel("chassis_speed_m_s"),
            origin,
            plot_size);
        origin.y += plot_size.y + 6.0f;
    }
    if (show_sensor)
    {
        drawSeriesPlot(
            draw_list,
            "sensor_distance_front_m",
            g_observer.channel("sensor_distance_front_m"),
            origin,
            plot_size);
    }

    ImGui::Dummy(ImVec2(0.0f, 4.0f));
    ImGui::Separator();
    ImGui::Text("Latest values");
    if (ImGui::BeginTable("##observation_values", 2, ImGuiTableFlags_RowBg))
    {
        ImGui::TableSetupColumn("channel");
        ImGui::TableSetupColumn("value");
        ImGui::TableHeadersRow();
        for (const std::string& name : g_observer.channelNames())
        {
            const srp::editor::TimeSeries& series = g_observer.channel(name);
            const std::optional<double> latest = series.latestValue();
            if (!latest.has_value())
            {
                continue;
            }
            ImGui::TableNextRow();
            ImGui::TableSetColumnIndex(0);
            ImGui::TextUnformatted(name.c_str());
            ImGui::TableSetColumnIndex(1);
            ImGui::Text("%.4f", *latest);
        }
        ImGui::EndTable();
    }

    ImGui::End();
}

void drawRecordingPanel()
{
    ImGui::SetNextWindowPos(ImVec2(560.0f, 700.0f), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize(ImVec2(330.0f, 190.0f), ImGuiCond_FirstUseEver);
    ImGui::Begin("Recording");

    if (g_recorder.recording())
    {
        if (ImGui::Button("Stop Recording"))
        {
            g_recorder.end();
        }
        ImGui::SameLine();
        ImGui::Text("recording... frames=%zu", g_recorder.size());
    }
    else
    {
        if (ImGui::Button("Record"))
        {
            g_playback_active = false;
            g_playback.stop();
            g_recorder.begin();
        }
        ImGui::SameLine();
        ImGui::Text("frames=%zu", g_recorder.size());
    }

    if (ImGui::Button("Save"))
    {
        std::string error;
        if (!g_recorder.save("assets/recordings/last_sim.json", error))
        {
            srp::core::logError("recording save failed: " + error);
        }
    }
    ImGui::SameLine();
    if (ImGui::Button("Load"))
    {
        std::string error;
        if (!g_recorder.load("assets/recordings/last_sim.json", error))
        {
            srp::core::logError("recording load failed: " + error);
        }
        else
        {
            g_playback.setSnapshots(g_recorder.snapshots());
            g_playback_active = false;
            g_playback.stop();
        }
    }

    ImGui::Separator();
    if (g_playback.hasSnapshots())
    {
        float playback_time = static_cast<float>(g_playback.time());
        const float duration = static_cast<float>(g_playback.duration());
        if (ImGui::SliderFloat(
                "Time",
                &playback_time,
                0.0f,
                duration,
                "%.2f s"))
        {
            g_playback.seek(playback_time);
        }

        if (g_playback_active)
        {
            if (ImGui::Button("Pause"))
            {
                g_playback.pause();
            }
        }
        else
        {
            if (ImGui::Button("Play"))
            {
                g_playback.play();
                g_playback_active = true;
                g_sim_control.pause();
            }
        }
        ImGui::SameLine();
        if (ImGui::Button("Stop"))
        {
            g_playback.stop();
            g_playback_active = false;
        }
        ImGui::SameLine();
        if (ImGui::Button("Live"))
        {
            g_playback_active = false;
            g_playback.stop();
        }
        ImGui::Text(
            "playback %.2f / %.2f s (%s)",
            g_playback.time(),
            duration,
            g_playback.isPlaying() ? "playing" : "paused");
    }
    else
    {
        ImGui::TextDisabled("record or load a simulation first");
    }

    ImGui::End();
}

srp::editor::ShapeKind shapeKindFromName(const std::string& kind)
{
    if (kind == "sphere")
    {
        return srp::editor::ShapeKind::kSphere;
    }
    if (kind == "cylinder")
    {
        return srp::editor::ShapeKind::kCylinder;
    }
    return srp::editor::ShapeKind::kBox;
}

void applyCourse(
    const srp::editor::CourseDefinition& course,
    srp::editor::SceneEditor& scene_editor)
{
    g_playback_active = false;
    g_playback.stop();
    g_recorder.clear();
    g_sim_control.pause();
    g_sim_control.resetCounters();
    g_observer.clear();
    g_simulation_ticks = 0;

    scene_editor.clearObjects();
    for (const srp::editor::SceneObjectSpec& spec : course.initial_bodies)
    {
        scene_editor.addBody(shapeKindFromName(spec.kind), spec.position);
    }

    if (course.entity == "drone")
    {
        g_demo.drone = std::make_unique<srp::scripting::DroneClosedLoopDemo>();
        g_demo.car.reset();
        g_demo.arm.reset();
        g_demo.kind = "drone";
    }
    else if (course.entity == "arm")
    {
        g_demo.arm = std::make_unique<srp::scripting::ArmClosedLoopDemo>();
        g_demo.car.reset();
        g_demo.drone.reset();
        g_demo.kind = "arm";
    }
    else
    {
        g_demo.car = std::make_unique<srp::scripting::CarClosedLoopDemo>();
        g_demo.drone.reset();
        g_demo.arm.reset();
        g_demo.kind = "car";
    }

    std::string script_text;
    if (!course.script.empty())
    {
        std::ifstream stream(course.script);
        if (stream.is_open())
        {
            std::ostringstream buffer;
            buffer << stream.rdbuf();
            script_text = buffer.str();
        }
    }

    std::string error;
    if (!script_text.empty() && !g_demo.loadScript(script_text, error))
    {
        srp::core::logError("course script failed: " + error);
    }

    g_demo.host().clearSensorOverrides();
    if (course.id == "line_follower")
    {
        g_demo.host().setSensorOverride(
            11,
            []
            {
                return std::optional<double>(g_left_sensor.distance());
            });
        g_demo.host().setSensorOverride(
            12,
            []
            {
                return std::optional<double>(g_right_sensor.distance());
            });
    }

    g_script_status = "course loaded: " + course.title;
}

void drawCoursesPanel()
{
    if (g_course_catalog_ptr == nullptr || g_scene_editor == nullptr)
    {
        return;
    }

    ImGui::SetNextWindowPos(ImVec2(900.0f, 440.0f), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize(ImVec2(520.0f, 350.0f), ImGuiCond_FirstUseEver);
    ImGui::Begin("Courses");

    static const srp::editor::CourseDefinition* selected = nullptr;
    const std::vector<srp::editor::CourseDefinition>& courses =
        g_course_catalog_ptr->courses();
    if (courses.empty())
    {
        ImGui::TextDisabled("no course files found in assets/courses");
    }

    for (const srp::editor::CourseDefinition& course : courses)
    {
        const bool is_selected = selected == &course;
        if (ImGui::Selectable(course.title.c_str(), is_selected))
        {
            selected = &course;
        }
    }

    if (selected != nullptr)
    {
        ImGui::Separator();
        ImGui::TextWrapped("%s", selected->description.c_str());
        ImGui::TextWrapped("Objective: %s", selected->objective.c_str());
        ImGui::Text(
            "Entity: %s | Script: %s",
            selected->entity.c_str(),
            selected->script.c_str());
        if (ImGui::Button("Load Course"))
        {
            applyCourse(*selected, *g_scene_editor);
        }
    }

    ImGui::End();
}

void drawScriptPanel()
{
    if (g_script_editor == nullptr)
    {
        return;
    }

    const bool dirty = g_script_editor->dirty();
    ImGui::SetNextWindowPos(ImVec2(900.0f, 10.0f), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize(ImVec2(560.0f, 420.0f), ImGuiCond_FirstUseEver);
    ImGui::Begin(dirty ? "Script*" : "Script");

    static char path_buffer[512]{};
    const std::string current_path = g_script_editor->path().string();
    std::snprintf(
        path_buffer,
        sizeof(path_buffer),
        "%s",
        current_path.c_str());

    ImGui::InputText("Path", path_buffer, sizeof(path_buffer));
    if (ImGui::Button("Open"))
    {
        std::string error;
        if (!g_script_editor->load(path_buffer, error))
        {
            g_script_status = "open failed: " + error;
        }
        else
        {
            g_script_status = "opened " + std::string(path_buffer);
        }
    }
    ImGui::SameLine();
    if (ImGui::Button("Save"))
    {
        if (!g_script_editor->path().empty())
        {
            std::string error;
            if (!g_script_editor->saveCurrent(error))
            {
                g_script_status = "save failed: " + error;
            }
            else
            {
                g_script_status = "saved " + g_script_editor->path().string();
            }
        }
        else
        {
            g_script_status = "set a path before saving";
        }
    }
    ImGui::SameLine();
    if (ImGui::Button("Run"))
    {
        if (g_demo.has())
        {
            std::string run_error;
            if (g_demo.loadScript(g_script_editor->text(), run_error))
            {
                g_script_status = "script loaded and running";
            }
            else
            {
                g_script_status = "script error: " + run_error;
            }
        }
    }
    ImGui::SameLine();
    if (ImGui::Button("Reload from file"))
    {
        std::string error;
        if (!g_script_editor->path().empty() &&
            g_script_editor->load(g_script_editor->path(), error))
        {
            g_script_status = "reloaded from " + g_script_editor->path().string();
        }
        else
        {
            g_script_status = "reload failed: " + error;
        }
    }

    ImGui::Text("Ctrl+S saves the current file.");
    ImGui::Separator();

    static std::string text_buffer;
    text_buffer = g_script_editor->text();
    text_buffer.push_back('\0');
    const ImVec2 text_size = ImGui::GetContentRegionAvail();
    if (ImGui::InputTextMultiline(
            "##script_source",
            text_buffer.data(),
            static_cast<int>(text_buffer.size()),
            ImVec2(text_size.x, std::max(120.0f, text_size.y - 60.0f)),
            ImGuiInputTextFlags_AllowTabInput))
    {
        g_script_editor->setText(std::string(text_buffer.data()));
    }

    ImGui::Separator();
    ImGui::TextWrapped("%s", g_script_status.c_str());

    if (ImGui::IsWindowFocused() &&
        ImGui::IsKeyChordPressed(ImGuiMod_Ctrl | ImGuiKey_S))
    {
        std::string error;
        if (g_script_editor->saveCurrent(error))
        {
            g_script_status = "saved " + g_script_editor->path().string();
        }
        else
        {
            g_script_status = "save failed: " + error;
        }
    }

    ImGui::End();
}

void drawScenePanel()
{
    if (g_scene_editor == nullptr)
    {
        return;
    }

    ImGui::SetNextWindowPos(ImVec2(10.0f, 10.0f), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize(ImVec2(300.0f, 420.0f), ImGuiCond_FirstUseEver);
    ImGui::Begin("Scene");

    ImGui::Text("Objects: %zu", g_scene_editor->entries().size());
    if (ImGui::Button("Add Box"))
    {
        g_scene_editor->addBody(srp::editor::ShapeKind::kBox);
    }
    ImGui::SameLine();
    if (ImGui::Button("Add Sphere"))
    {
        g_scene_editor->addBody(srp::editor::ShapeKind::kSphere);
    }
    ImGui::SameLine();
    if (ImGui::Button("Add Cylinder"))
    {
        g_scene_editor->addBody(srp::editor::ShapeKind::kCylinder);
    }

    ImGui::Checkbox("Move mode (drag on ground)", &g_move_mode);
    ImGui::Text("Left click: select  |  Right drag: orbit  |  Wheel: zoom");
    ImGui::Text("Delete: remove  |  Esc: deselect");
    ImGui::Separator();

    for (const srp::editor::BodyEntry& entry : g_scene_editor->entries())
    {
        const bool is_selected = g_scene_editor->selected() == entry.id;
        if (ImGui::Selectable(entry.name.c_str(), is_selected))
        {
            g_scene_editor->select(entry.id);
        }
    }

    const std::optional<srp::physics::BodyId> selected = g_scene_editor->selected();
    if (selected.has_value())
    {
        srp::editor::BodyEntry* entry = g_scene_editor->entry(*selected);
        if (entry != nullptr)
        {
            ImGui::Separator();
            char name_buffer[128]{};
            std::snprintf(name_buffer, sizeof(name_buffer), "%s", entry->name.c_str());
            if (ImGui::InputText("Name", name_buffer, sizeof(name_buffer)))
            {
                entry->name = name_buffer;
            }

            const srp::physics::RigidBodyState* body =
                g_scene_editor->world().body(*selected);
            if (body != nullptr)
            {
                ImGui::Text(
                    "Position: (%.2f, %.2f, %.2f)",
                    body->position.x,
                    body->position.y,
                    body->position.z);
                ImGui::Text("Velocity: (%.2f, %.2f, %.2f)",
                    body->linear_velocity.x,
                    body->linear_velocity.y,
                    body->linear_velocity.z);
            }

            if (ImGui::Button("Delete"))
            {
                g_scene_editor->removeBody(*selected);
            }
        }
    }

    ImGui::End();
}

LRESULT CALLBACK windowProc(HWND window, UINT message, WPARAM w_param, LPARAM l_param)
{
    if (ImGui_ImplWin32_WndProcHandler(window, message, w_param, l_param))
    {
        return true;
    }

    switch (message)
    {
    case WM_PAINT:
        ValidateRect(window, nullptr);
        return 0;

    case WM_ERASEBKGND:
        return 1;

    case WM_LBUTTONDOWN:
        handleViewportClick(window, GET_X_LPARAM(l_param), GET_Y_LPARAM(l_param));
        return 0;

    case WM_LBUTTONUP:
        g_move_dragging = false;
        ReleaseCapture();
        return 0;

    case WM_RBUTTONDOWN:
        g_camera_orbiting = true;
        g_last_mouse_x = GET_X_LPARAM(l_param);
        g_last_mouse_y = GET_Y_LPARAM(l_param);
        SetCapture(window);
        return 0;

    case WM_RBUTTONUP:
        g_camera_orbiting = false;
        ReleaseCapture();
        return 0;

    case WM_MOUSEMOVE:
    {
        const int x = GET_X_LPARAM(l_param);
        const int y = GET_Y_LPARAM(l_param);

        if (g_camera_orbiting)
        {
            const double dx = static_cast<double>(x - g_last_mouse_x);
            const double dy = static_cast<double>(y - g_last_mouse_y);
            g_last_mouse_x = x;
            g_last_mouse_y = y;
            g_camera_yaw += dx * 0.01;
            g_camera_pitch += dy * 0.01;
            g_camera_pitch = std::clamp(g_camera_pitch, -1.5, 1.5);
        }
        else
        {
            handleViewportDrag(window, x, y);
        }
        return 0;
    }

    case WM_MOUSEWHEEL:
    {
        const short wheel_delta = GET_WHEEL_DELTA_WPARAM(w_param);
        g_camera_distance *= std::pow(0.9, wheel_delta / 120.0);
        g_camera_distance = std::clamp(g_camera_distance, 1.5, 30.0);
        return 0;
    }

    case WM_KEYDOWN:
        if (g_scene_editor != nullptr && !ImGui::GetIO().WantCaptureKeyboard)
        {
            const std::optional<srp::physics::BodyId> selected =
                g_scene_editor->selected();
            if (w_param == VK_SPACE)
            {
                g_sim_control.toggle();
            }
            else if (w_param == VK_RIGHT)
            {
                g_sim_control.requestStep();
            }
            else if (w_param == VK_DELETE && selected.has_value())
            {
                g_scene_editor->removeBody(*selected);
            }
            else if (w_param == VK_ESCAPE)
            {
                g_scene_editor->deselect();
                if (g_circuit_editor != nullptr)
                {
                    g_circuit_editor->cancelWire();
                }
                g_circuit_placement.reset();
            }
        }
        return 0;

    case WM_DESTROY:
        PostQuitMessage(0);
        return 0;

    default:
        return DefWindowProcW(window, message, w_param, l_param);
    }
}

}  // namespace

int WINAPI WinMain(
    HINSTANCE instance,
    HINSTANCE,
    LPSTR command_line,
    int show_command)
{
    const auto config = srp::core::loadAppConfig("assets/config/app.json");
    srp::core::initLogging(config.logging.level);
    srp::core::logInfo("SRPlatform starting");

    const std::wstring window_title(
        config.window.title.begin(),
        config.window.title.end());

    WNDCLASSW window_class{};
    window_class.lpfnWndProc = windowProc;
    window_class.hInstance = instance;
    window_class.lpszClassName = kWindowClassName;
    window_class.hCursor = LoadCursorW(nullptr, reinterpret_cast<LPCWSTR>(IDC_ARROW));
    window_class.hbrBackground = nullptr;
    window_class.style = CS_OWNDC;

    if (RegisterClassW(&window_class) == 0)
    {
        srp::core::logError("failed to register window class");
        return 1;
    }

    HWND window = CreateWindowExW(
        0,
        kWindowClassName,
        window_title.c_str(),
        WS_OVERLAPPEDWINDOW,
        CW_USEDEFAULT,
        CW_USEDEFAULT,
        config.window.width,
        config.window.height,
        nullptr,
        nullptr,
        instance,
        nullptr);

    if (window == nullptr)
    {
        srp::core::logError("failed to create window");
        return 1;
    }

    HDC device_context = GetDC(window);
    if (device_context == nullptr)
    {
        srp::core::logError("failed to get device context");
        DestroyWindow(window);
        return 1;
    }

    PIXELFORMATDESCRIPTOR pixel_format_descriptor{};
    pixel_format_descriptor.nSize = sizeof(PIXELFORMATDESCRIPTOR);
    pixel_format_descriptor.nVersion = 1;
    pixel_format_descriptor.dwFlags = PFD_DRAW_TO_WINDOW | PFD_SUPPORT_OPENGL | PFD_DOUBLEBUFFER;
    pixel_format_descriptor.iPixelType = PFD_TYPE_RGBA;
    pixel_format_descriptor.cColorBits = 32;
    pixel_format_descriptor.cDepthBits = 24;
    pixel_format_descriptor.iLayerType = PFD_MAIN_PLANE;

    const int pixel_format = ChoosePixelFormat(device_context, &pixel_format_descriptor);
    if (pixel_format == 0 ||
        SetPixelFormat(device_context, pixel_format, &pixel_format_descriptor) == FALSE)
    {
        srp::core::logError("failed to set OpenGL pixel format");
        ReleaseDC(window, device_context);
        DestroyWindow(window);
        return 1;
    }

    HGLRC gl_context = wglCreateContext(device_context);
    if (gl_context == nullptr || wglMakeCurrent(device_context, gl_context) == FALSE)
    {
        srp::core::logError("failed to create OpenGL context");
        if (gl_context != nullptr)
        {
            wglDeleteContext(gl_context);
        }
        ReleaseDC(window, device_context);
        DestroyWindow(window);
        return 1;
    }

    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGui::GetIO().IniFilename = nullptr;
    ImGui::StyleColorsDark();
    ImGui_ImplWin32_Init(window);
    ImGui_ImplOpenGL2_Init();

    srp::editor::SceneEditor scene_editor;
    g_scene_editor = &scene_editor;

    srp::editor::CircuitEditor circuit_editor;
    g_circuit_editor = &circuit_editor;

    srp::bridge::DistanceSensorParameters sensor_parameters;
    sensor_parameters.max_range_m = 4.0;
    sensor_parameters.beam_axis_local = srp::math::Vec3(1.0, 0.0, 0.0);
    g_front_sensor = srp::bridge::DistanceSensorModel(sensor_parameters);

    srp::bridge::DistanceSensorParameters side_sensor_parameters;
    side_sensor_parameters.max_range_m = 4.0;
    side_sensor_parameters.beam_axis_local = srp::math::Vec3(
        std::cos(kSideBeamAngle),
        0.0,
        -std::sin(kSideBeamAngle));
    g_left_sensor = srp::bridge::DistanceSensorModel(side_sensor_parameters);
    side_sensor_parameters.beam_axis_local = srp::math::Vec3(
        std::cos(kSideBeamAngle),
        0.0,
        std::sin(kSideBeamAngle));
    g_right_sensor = srp::bridge::DistanceSensorModel(side_sensor_parameters);

    std::string course_error;
    if (!g_course_catalog.loadDirectory("assets/courses", course_error))
    {
        srp::core::logError("failed to load courses: " + course_error);
    }
    g_course_catalog_ptr = &g_course_catalog;

    // Parse "--course <id>" from the command line for testing/launching.
    std::string course_id = "rc_car";
    if (command_line != nullptr)
    {
        std::istringstream arguments(command_line);
        std::string argument;
        while (arguments >> argument)
        {
            if (argument == "--course" && (arguments >> argument))
            {
                course_id = argument;
            }
        }
    }

    const srp::editor::CourseDefinition* default_course =
        g_course_catalog.find(course_id);
    if (default_course != nullptr)
    {
        applyCourse(*default_course, scene_editor);
        g_sim_control.play();
    }
    else
    {
        g_demo.car = std::make_unique<srp::scripting::CarClosedLoopDemo>();
        g_demo.kind = "car";
    }

    srp::editor::ScriptEditor script_editor;
    g_script_editor = &script_editor;
    std::string script_error;
    if (!script_editor.load("assets/scripts/car_controller.lua", script_error))
    {
        srp::core::logError("failed to open example script: " + script_error);
    }
    g_script_status = script_editor.path().empty()
        ? "no script loaded"
        : "loaded " + script_editor.path().string();

    srp::core::logInfo(
        "SRPlatform editor ready (left click select, right drag orbit, wheel zoom)");

    ShowWindow(window, show_command);
    UpdateWindow(window);

    srp::core::FixedStepLoop fixed_loop(
        config.simulation.fixed_dt,
        config.simulation.max_steps_per_frame);
    fixed_loop.reset(srp::core::FixedStepLoop::Clock::now());

    MSG message{};
    bool running = true;
    while (running)
    {
        while (PeekMessageW(&message, nullptr, 0, 0, PM_REMOVE))
        {
            if (message.message == WM_QUIT)
            {
                running = false;
                break;
            }

            TranslateMessage(&message);
            DispatchMessageW(&message);
        }

        if (!running)
        {
            break;
        }

        const auto now = srp::core::FixedStepLoop::Clock::now();
        const auto update = fixed_loop.advance(now);

        if (g_playback_active)
        {
            if (g_has_last_frame_time)
            {
                const double frame_duration =
                    std::chrono::duration<double>(now - g_last_frame_time).count();
                g_playback.advance(frame_duration);
            }
        }
        g_last_frame_time = now;
        g_has_last_frame_time = true;

        const std::size_t steps_to_run = g_playback_active
            ? 0u
            : g_sim_control.stepsToRun(config.simulation.max_steps_per_frame);
        for (std::size_t i = 0; i < steps_to_run; ++i)
        {
            scene_editor.world().step(config.simulation.fixed_dt);
            g_demo.step(config.simulation.fixed_dt);
            ++g_simulation_ticks;

            if (g_recorder.recording())
            {
                g_recorder.record(
                    g_demo.elapsedTime(),
                    captureBodySnapshots(scene_editor));
            }

            if (g_observation_enabled)
            {
                if (g_demo.kind == "car" && g_demo.car != nullptr)
                {
                    srp::editor::sampleCarTelemetry(
                        g_demo.car->car(),
                        scene_editor.world(),
                        config.simulation.fixed_dt,
                        g_observer);

                    const srp::physics::RigidBodyState* chassis =
                        g_demo.car->car().chassisBody();
                    if (chassis != nullptr)
                    {
                        const double sensor_y = 0.25;

                        g_front_sensor.setPose(
                            chassis->position + srp::math::Vec3(0.0, sensor_y, 0.0),
                            chassis->orientation);
                        g_front_sensor.update(scene_editor.world());
                        g_observer.recordSensor(
                            "distance_front_m",
                            g_front_sensor.distance());
                        g_observer.recordSensor(
                            "distance_detected",
                            g_front_sensor.detected() ? 1.0 : 0.0);

                        srp::math::Vec3 beam_left(
                            std::cos(kSideBeamAngle),
                            0.0,
                            -std::sin(kSideBeamAngle));
                        srp::math::Vec3 beam_right(
                            std::cos(kSideBeamAngle),
                            0.0,
                            std::sin(kSideBeamAngle));

                        g_left_sensor.setPose(
                            chassis->position + srp::math::Vec3(0.0, sensor_y, -0.25),
                            chassis->orientation);
                        g_left_sensor.update(scene_editor.world());
                        g_right_sensor.setPose(
                            chassis->position + srp::math::Vec3(0.0, sensor_y, 0.25),
                            chassis->orientation);
                        g_right_sensor.update(scene_editor.world());
                        g_observer.recordSensor(
                            "distance_left_m",
                            g_left_sensor.distance());
                        g_observer.recordSensor(
                            "distance_right_m",
                            g_right_sensor.distance());
                    }
                }
                else if (g_demo.kind == "drone" && g_demo.drone != nullptr)
                {
                    srp::editor::sampleDroneTelemetry(
                        g_demo.drone->drone(),
                        g_observer);
                }
                else if (g_demo.kind == "arm" && g_demo.arm != nullptr)
                {
                    srp::editor::sampleArmTelemetry(
                        g_demo.arm->arm(),
                        g_observer);
                }
            }
        }

        ++g_render_frames;

        RECT client_rect{};
        GetClientRect(window, &client_rect);

        renderScene(
            client_rect.right - client_rect.left,
            client_rect.bottom - client_rect.top,
            scene_editor);

        ImGui_ImplOpenGL2_NewFrame();
        ImGui_ImplWin32_NewFrame();
        ImGui::NewFrame();

        drawScenePanel();
        drawCircuitPanel();
        drawObservationPanel();
        drawRecordingPanel();
        drawCoursesPanel();
        drawScriptPanel();

        ImGui::SetNextWindowPos(ImVec2(10.0f, 810.0f), ImGuiCond_FirstUseEver);
        ImGui::SetNextWindowSize(ImVec2(320.0f, 90.0f), ImGuiCond_FirstUseEver);
        ImGui::Begin("Simulation");
        const bool sim_running = g_sim_control.isRunning();
        if (ImGui::Button(sim_running ? "Pause" : "Play"))
        {
            g_sim_control.toggle();
        }
        ImGui::SameLine();
        if (ImGui::Button("Step"))
        {
            g_sim_control.requestStep();
        }
        ImGui::SameLine();
        float speed_value = static_cast<float>(g_sim_control.speed());
        if (ImGui::SliderFloat(
                "Speed",
                &speed_value,
                0.0f,
                4.0f,
                "%.1fx"))
        {
            g_sim_control.setSpeed(speed_value);
        }
        ImGui::Text(
            "%s | steps: %llu | Space pause/play, Right step",
            sim_running ? "running" : "paused",
            g_sim_control.stepsTaken());
        ImGui::End();

        ImGui::Render();
        ImGui_ImplOpenGL2_RenderDrawData(ImGui::GetDrawData());
        SwapBuffers(device_context);

        double status_value = 0.0;
        std::wstring status_name = L"idle";
        if (g_demo.kind == "car" && g_demo.car != nullptr)
        {
            const srp::physics::RigidBodyState* chassis =
                g_demo.car->car().chassisBody();
            status_value = chassis != nullptr ? chassis->position.x : 0.0;
            status_name = L"car_x";
        }
        else if (g_demo.kind == "drone" && g_demo.drone != nullptr)
        {
            status_value = g_demo.drone->drone().altitude();
            status_name = L"altitude";
        }
        else if (g_demo.kind == "arm" && g_demo.arm != nullptr)
        {
            status_value = g_demo.arm->arm().endEffectorPosition().y;
            status_name = L"effector_y";
        }

        std::wstring title =
            L"SRPlatform | " +
            std::wstring(sim_running ? L"running" : L"paused") +
            L" | sim=" + std::to_wstring(g_simulation_ticks) +
            L" frames=" + std::to_wstring(g_render_frames) +
            L" alpha=" + std::to_wstring(update.alpha) +
            L" " + status_name + L"=" + std::to_wstring(status_value);
        SetWindowTextW(window, title.c_str());

        Sleep(1);
    }

    g_scene_editor = nullptr;
    g_circuit_editor = nullptr;
    g_script_editor = nullptr;
    g_course_catalog_ptr = nullptr;
    ImGui_ImplOpenGL2_Shutdown();
    ImGui_ImplWin32_Shutdown();
    ImGui::DestroyContext();

    wglMakeCurrent(nullptr, nullptr);
    wglDeleteContext(gl_context);
    ReleaseDC(window, device_context);
    DestroyWindow(window);

    srp::core::logInfo("SRPlatform stopped");
    return static_cast<int>(message.wParam);
}
