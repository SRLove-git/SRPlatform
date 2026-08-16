#define WIN32_LEAN_AND_MEAN
#include <windows.h>

namespace
{

constexpr wchar_t kWindowClassName[] = L"SRPlatformWindow";

LRESULT CALLBACK windowProc(HWND window, UINT message, WPARAM w_param, LPARAM l_param)
{
    switch (message)
    {
    case WM_PAINT:
    {
        PAINTSTRUCT paint{};
        HDC device_context = BeginPaint(window, &paint);
        FillRect(device_context, &paint.rcPaint, static_cast<HBRUSH>(GetStockObject(WHITE_BRUSH)));
        EndPaint(window, &paint);
        return 0;
    }

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
    WNDCLASSW window_class{};
    window_class.lpfnWndProc = windowProc;
    window_class.hInstance = instance;
    window_class.lpszClassName = kWindowClassName;
    window_class.hCursor = LoadCursorW(nullptr, reinterpret_cast<LPCWSTR>(IDC_ARROW));
    window_class.hbrBackground = static_cast<HBRUSH>(GetStockObject(WHITE_BRUSH));

    if (RegisterClassW(&window_class) == 0)
    {
        return 1;
    }

    HWND window = CreateWindowExW(
        0,
        kWindowClassName,
        L"SRPlatform",
        WS_OVERLAPPEDWINDOW,
        CW_USEDEFAULT,
        CW_USEDEFAULT,
        800,
        600,
        nullptr,
        nullptr,
        instance,
        nullptr);

    if (window == nullptr)
    {
        return 1;
    }

    ShowWindow(window, show_command);
    UpdateWindow(window);

    MSG message{};
    while (GetMessageW(&message, nullptr, 0, 0) > 0)
    {
        TranslateMessage(&message);
        DispatchMessageW(&message);
    }

    return static_cast<int>(message.wParam);
}
