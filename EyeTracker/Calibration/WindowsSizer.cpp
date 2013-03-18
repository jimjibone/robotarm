#include "WindowsSizer.h"

WindowsSizer::WindowsSizer()
{
    RECT desktop;
    HWND hDesktop = GetDesktopWindow();
    GetWindowRect(hDesktop, &desktop);
    Width = desktop.right- desktop.left;
    Height = desktop.bottom - desktop.top;
}

WindowsSizer::~WindowsSizer()
{
    //dtor
}

//Code got from http://stackoverflow.com/questions/6512094/how-to-display-an-image-in-full-screen-borderless-window-in-opencv
void WindowsSizer::SetWindowToFullScreen(char* WindowName)
{
    HWND win_handle = FindWindow(0, WindowName);
    if (!win_handle)
    {
        printf("Failed FindWindow\n");
        return;
    }

    // Resize
    unsigned int flags = (SWP_SHOWWINDOW | SWP_NOSIZE | SWP_NOMOVE | SWP_NOZORDER);
    flags &= ~SWP_NOSIZE;
    unsigned int x = 0;
    unsigned int y = 0;
    unsigned int w = Width;
    unsigned int h = Height;
    SetWindowPos(win_handle, HWND_NOTOPMOST, x, y, w, h, flags);

    // Borderless
    SetWindowLong(win_handle, GWL_STYLE, GetWindowLong(win_handle, GWL_EXSTYLE) | WS_EX_TOPMOST);
    ShowWindow(win_handle, SW_SHOW);
}

int WindowsSizer::GetWidth()
{
    return Width;
}

int WindowsSizer::GetHeight()
{
    return Height;
}
