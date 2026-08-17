#define WIN32_LEAN_AND_MEAN
#include "core/config/app_config.hpp"
#include "core/loop/fixed_step_loop.hpp"
#include "core/logging.hpp"
#include "editor/scene_editor.hpp"
#include "rendering/debug_draw.hpp"
#include "scripting/car_closed_loop_demo.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <string>
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

constexpr const char* kCarControllerScript =
    "elapsed = 0\n"
    "function update(dt)\n"
    "    elapsed = elapsed + dt\n"
    "    if elapsed < 2.0 then\n"
    "        set_motor(1, 1.0)\n"
    "        set_servo(1, 0.0)\n"
    "    elseif elapsed < 4.0 then\n"
    "        set_motor(1, 0.0)\n"
    "        set_servo(1, 0.6)\n"
    "    elseif elapsed < 6.0 then\n"
    "        set_motor(1, 0.8)\n"
    "        set_servo(1, -0.6)\n"
    "    elseif elapsed < 8.0 then\n"
    "        set_motor(1, -0.5)\n"
    "        set_servo(1, 0.0)\n"
    "    else\n"
    "        elapsed = 0.0\n"
    "    end\n"
    "end\n";

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
    const srp::editor::SceneEditor& scene_editor,
    const srp::scripting::CarClosedLoopDemo& car_demo)
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

    const srp::physics::RigidBodyState* chassis = car_demo.car().chassisBody();
    if (chassis != nullptr)
    {
        glColor3d(0.2, 0.9, 0.4);
        srp::rendering::drawCollisionShape(
            car_demo.car().chassisShape(),
            chassis->position,
            chassis->orientation);
    }

    const srp::physics::RigidBodyState* wheel = car_demo.car().wheelBody();
    if (wheel != nullptr)
    {
        glColor3d(0.95, 0.7, 0.2);
        srp::rendering::drawCollisionShape(
            car_demo.car().wheelShape(),
            wheel->position,
            wheel->orientation);
    }

    glColor3d(1.0, 0.2, 0.2);
    for (const srp::physics::Contact& contact : world.contacts())
    {
        srp::rendering::drawContactPoint(contact.point);
    }
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
            if (w_param == VK_DELETE && selected.has_value())
            {
                g_scene_editor->removeBody(*selected);
            }
            else if (w_param == VK_ESCAPE)
            {
                g_scene_editor->deselect();
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

int WINAPI WinMain(HINSTANCE instance, HINSTANCE, LPSTR, int show_command)
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
    scene_editor.addBody(srp::editor::ShapeKind::kBox);
    scene_editor.addBody(srp::editor::ShapeKind::kSphere);

    srp::scripting::CarClosedLoopDemo car_demo;
    if (!car_demo.loadScript("controller", kCarControllerScript))
    {
        srp::core::logError("failed to load car controller script");
    }

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

        for (std::size_t i = 0; i < update.steps; ++i)
        {
            scene_editor.world().step(config.simulation.fixed_dt);
            car_demo.step(config.simulation.fixed_dt);
            ++g_simulation_ticks;
        }

        ++g_render_frames;

        RECT client_rect{};
        GetClientRect(window, &client_rect);

        renderScene(
            client_rect.right - client_rect.left,
            client_rect.bottom - client_rect.top,
            scene_editor,
            car_demo);

        ImGui_ImplOpenGL2_NewFrame();
        ImGui_ImplWin32_NewFrame();
        ImGui::NewFrame();

        drawScenePanel();

        ImGui::Begin("Tool UI");
        ImGui::Text("Simulation ticks: %llu", g_simulation_ticks);
        ImGui::Text("Render frames: %llu", g_render_frames);
        ImGui::Text("Fixed steps this frame: %zu", update.steps);
        ImGui::End();

        ImGui::Render();
        ImGui_ImplOpenGL2_RenderDrawData(ImGui::GetDrawData());
        SwapBuffers(device_context);

        const double car_x = car_demo.car().chassisBody() != nullptr
            ? car_demo.car().chassisBody()->position.x
            : 0.0;

        std::wstring title =
            L"SRPlatform | sim=" + std::to_wstring(g_simulation_ticks) +
            L" frames=" + std::to_wstring(g_render_frames) +
            L" alpha=" + std::to_wstring(update.alpha) +
            L" car_x=" + std::to_wstring(car_x);
        SetWindowTextW(window, title.c_str());

        Sleep(1);
    }

    g_scene_editor = nullptr;
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
