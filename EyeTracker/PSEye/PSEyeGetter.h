#ifndef PSEYEGETTER_H
#define PSEYEGETTER_H

#include "CLEyeMulticam.h"

class PSEyeGetter
{
public:
    PSEyeGetter();
    PSEyeGetter(CLEyeCameraColorMode ColourMode, CLEyeCameraResolution Resolution, float FrameRate);
    ~PSEyeGetter();

    bool FindCam(CLEyeCameraColorMode ColourMode, CLEyeCameraResolution Resolution, float FrameRate);
    bool GetFrame();

    PBYTE CurrentColours;

    int ImageWidth();
    int ImageHeight();
    int ImageDepth();
    int ImageStride();
    int TotalPixels();

    float FrameRate();

    bool CamIsReady();
protected:

private:
    CLEyeCameraColorMode Mode;
    CLEyeCameraResolution Res;

    CLEyeCameraInstance Cam;
    GUID guid;

    bool FindCam();
    int W, H, Size, Stride, ColourSize;
    float Rate;
    bool IsReady;
};

#endif // PSEYEGETTER_H
