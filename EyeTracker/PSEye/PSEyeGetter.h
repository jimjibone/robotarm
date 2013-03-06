#ifndef PSEYEGETTER_H
#define PSEYEGETTER_H

#include "CLEyeMulticam.h"
#include <iostream>

class PSEyeGetter
{
public:
    PSEyeGetter();
    PSEyeGetter(CLEyeCameraColorMode ColourMode, CLEyeCameraResolution Resolution, float FrameRate);
    ~PSEyeGetter();

    bool FindCam();
    bool GetFrame();

    PBYTE CurrentColours;

    bool IsReady;

    CLEyeCameraInstance Cam;
    GUID guid;

    int W, H, Size;
    float Rate;

    CLEyeCameraColorMode Mode;
    CLEyeCameraResolution Res;

protected:

private:
    bool BeenGot;
};

#endif // PSEYEGETTER_H
