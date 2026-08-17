#define WIN32_LEAN_AND_MEAN
#include "core/config/app_config.hpp"
#include "core/loop/fixed_step_loop.hpp"
#include "core/logging.hpp"

#include <chrono>
#include <cstdint>
#include <string>
#include <windows.h>

#include <GL/gl.h>
#include <GL/glu.h>

namespace
{

constexpr wchar_t kWindowClassName[] = L"SRPlatformWindow";

std::uint64_t g_simulation_ticks = 0;
std::uint64_t g_render_frames = 0;

void drawGround()
{
    glColor3f(0.25f, 0.28f, 0.32f);
    glBegin(GL_QUADS);
    glVertex3f(-5.0f, 0.0f, -5.0f);
    glVertex3f(5.0f, 0.0f, -5.0f);
    glVertex3f(5.0f, 0.0f, 5.0f);
    glVertex3f(-5.0f, 0.0f, 5.0f);
    glEnd();

    glColor3f(0.42f, 0.46f, 0.52f);
    glBegin(GL_LINES);
    for (int i = -5; i <= 5; ++i)
    {
        const float position = static_cast<float>(i);
        glVertex3f(position, 0.0f, -5.0f);
        glVertex3f(position, 0.0f, 5.0f);
        glVertex3f(-5.0f, 0.0f, position);
        glVertex3f(5.0f, 0.0f, position);
    }
    glEnd();
}

void drawBox()
{
    constexpr float min_x = -0.5f;
    constexpr float max_x = 0.5f;
    constexpr float min_y = 0.0f;
    constexpr float max_y = 1.0f;
    constexpr float min_z = -0.5f;
    constexpr float max_z = 0.5f;

    glColor3f(0.85f, 0.42f, 0.18f);
    glBegin(GL_QUADS);

    glVertex3f(min_x, min_y, max_z);
    glVertex3f(max_x, min_y, max_z);
    glVertex3f(max_x, max_y, max_z);
    glVertex3f(min_x, max_y, max_z);

    glVertex3f(min_x, min_y, min_z);
    glVertex3f(min_x, max_y, min_z);
    glVertex3f(max_x, max_y, min_z);
    glVertex3f(max_x, min_y, min_z);

    glVertex3f(min_x, max_y, min_z);
    glVertex3f(min_x, max_y, max_z);
    glVertex3f(max_x, max_y, max_z);
    glVertex3f(max_x, max_y, min_z);

    glVertex3f(min_x, min_y, min_z);
    glVertex3f(max_x, min_y, min_z);
    glVertex3f(max_x, min_y, max_z);
    glVertex3f(min_x, min_y, max_z);

    glVertex3f(min_x, min_y, min_z);
    glVertex3f(min_x, min_y, max_z);
    glVertex3f(min_x, max_y, max_z);
    glVertex3f(min_x, max_y, min_z);

    glVertex3f(max_x, min_y, min_z);
    glVertex3f(max_x, max_y, min_z);
    glVertex3f(max_x, max_y, max_z);
    glVertex3f(max_x, min_y, max_z);

    glEnd();

    glColor3f(1.0f, 1.0f, 1.0f);
    glBegin(GL_LINES);

    glVertex3f(min_x, min_y, min_z);
    glVertex3f(max_x, min_y, min_z);
    glVertex3f(min_x, min_y, max_z);
    glVertex3f(max_x, min_y, max_z);
    glVertex3f(min_x, max_y, min_z);
    glVertex3f(max_x, max_y, min_z);
    glVertex3f(min_x, max_y, max_z);
    glVertex3f(max_x, max_y, max_z);

    glVertex3f(min_x, min_y, min_z);
    glVertex3f(min_x, max_y, min_z);
    glVertex3f(max_x, min_y, min_z);
    glVertex3f(max_x, max_y, min_z);
    glVertex3f(min_x, min_y, max_z);
    glVertex3f(min_x, max_y, max_z);
    glVertex3f(max_x, min_y, max_z);
    glVertex3f(max_x, max_y, max_z);

    glVertex3f(min_x, min_y, min_z);
    glVertex3f(min_x, min_y, max_z);
    glVertex3f(max_x, min_y, min_z);
    glVertex3f(max_x, min_y, max_z);
    glVertex3f(min_x, max_y, min_z);
    glVertex3f(min_x, max_y, max_z);
    glVertex3f(max_x, max_y, min_z);
    glVertex3f(max_x, max_y, max_z);

    glEnd();
}

void renderScene(HDC device_context, int width, int height)
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

    drawGround();
    drawBox();

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
            ++g_simulation_ticks;
        }

        ++g_render_frames;

        RECT client_rect{};
        GetClientRect(window, &client_rect);
        renderScene(
            device_context,
            client_rect.right - client_rect.left,
            client_rect.bottom - client_rect.top);

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
