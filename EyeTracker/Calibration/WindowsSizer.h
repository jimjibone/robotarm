#ifndef WINDOWSSIZER_H
#define WINDOWSSIZER_H

#include <windows.h>
#include <iostream>

using namespace std;

class WindowsSizer
{
    public:
        WindowsSizer();
        virtual ~WindowsSizer();

        void SetWindowToFullScreen(char*);
        int GetWidth();
        int GetHeight();
    protected:
    private:
        int Width, Height;
};

#endif // WINDOWSSIZER_H
