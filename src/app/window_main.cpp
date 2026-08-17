#define WIN32_LEAN_AND_MEAN
#include "core/config/app_config.hpp"
#include "core/loop/fixed_step_loop.hpp"
#include "core/logging.hpp"
#include "physics/physics_world.hpp"
#include "rendering/debug_draw.hpp"

#include <chrono>
#include <cstdint>
#include <string>
#include <vector>
#include <windows.h>

#include <GL/gl.h>
#include <GL/glu.h>

namespace
{

constexpr wchar_t kWindowClassName[] = L"SRPlatformWindow";

std::uint64_t g_simulation_ticks = 0;
std::uint64_t g_render_frames = 0;

void renderScene(
    HDC device_context,
    int width,
    int height,
    const srp::physics::PhysicsWorld& world,
    const std::vector<srp::physics::BodyId>& body_ids)
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
    gluLookAt(
        2.0, 2.0, 5.0,
        0.0, 0.5, 0.0,
        0.0, 1.0, 0.0);

    glColor3d(0.35, 0.75, 1.0);
    for (const srp::physics::BodyId id : body_ids)
    {
        const srp::physics::RigidBodyState* body = world.body(id);
        const srp::physics::CollisionShape* shape = world.shape(id);
        if (body == nullptr || shape == nullptr)
        {
            continue;
        }

        srp::rendering::drawCollisionShape(
            *shape,
            body->position,
            body->orientation);
    }

    glColor3d(1.0, 0.2, 0.2);
    for (const srp::physics::Contact& contact : world.contacts())
    {
        srp::rendering::drawContactPoint(contact.point);
    }

    SwapBuffers(device_context);
}

LRESULT CALLBACK windowProc(HWND window, UINT message, WPARAM w_param, LPARAM l_param)
{
    switch (message)
    {
    case WM_PAINT:
        ValidateRect(window, nullptr);
        return 0;

    case WM_ERASEBKGND:
        return 1;

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

    srp::physics::PhysicsWorld world;
    std::vector<srp::physics::BodyId> body_ids;

    srp::physics::RigidBodyState ground_state;
    ground_state.type = srp::physics::RigidBodyType::kStatic;

    srp::physics::PlaneShape ground_plane;
    ground_plane.normal = srp::math::Vec3(0.0, 1.0, 0.0);
    body_ids.push_back(world.createBody(ground_state, ground_plane));

    srp::physics::RigidBodyState sphere_state;
    sphere_state.type = srp::physics::RigidBodyType::kDynamic;
    sphere_state.mass = 1.0;
    sphere_state.position = srp::math::Vec3(0.0, 2.0, 0.0);
    sphere_state.restitution = 0.3;

    srp::physics::SphereShape sphere_shape;
    sphere_shape.radius = 0.5;
    body_ids.push_back(world.createBody(sphere_state, sphere_shape));

    srp::physics::RigidBodyState box_state;
    box_state.type = srp::physics::RigidBodyType::kDynamic;
    box_state.mass = 1.0;
    box_state.position = srp::math::Vec3(0.0, 4.0, 0.0);
    box_state.friction = 0.6;

    srp::physics::BoxShape box_shape;
    box_shape.half_extents = srp::math::Vec3(0.5);
    body_ids.push_back(world.createBody(box_state, box_shape));

    srp::core::logInfo("SRPlatform window created");

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
            world.step(config.simulation.fixed_dt);
            ++g_simulation_ticks;
        }

        ++g_render_frames;

        RECT client_rect{};
        GetClientRect(window, &client_rect);
        renderScene(
            device_context,
            client_rect.right - client_rect.left,
            client_rect.bottom - client_rect.top,
            world,
            body_ids);

        std::wstring title =
            L"SRPlatform | sim=" + std::to_wstring(g_simulation_ticks) +
            L" frames=" + std::to_wstring(g_render_frames) +
            L" alpha=" + std::to_wstring(update.alpha);
        SetWindowTextW(window, title.c_str());

        Sleep(1);
    }

    wglMakeCurrent(nullptr, nullptr);
    wglDeleteContext(gl_context);
    ReleaseDC(window, device_context);
    DestroyWindow(window);

    srp::core::logInfo("SRPlatform stopped");
    return static_cast<int>(message.wParam);
}
