#include "../../MEngProjectFiles/PSEye/PSEyeGetter.h"

PSEyeGetter::PSEyeGetter()
{

}

PSEyeGetter::PSEyeGetter(CLEyeCameraColorMode ColourMode, CLEyeCameraResolution Resolution, float FrameRate)
{
    IsReady = false;
    Mode = ColourMode;
    Res = Resolution;
    Rate = FrameRate;
    switch(Mode)
    {
    case CLEYE_MONO_PROCESSED:
        Size = 1;
        break;
    case CLEYE_COLOR_PROCESSED:
        Size = 4;
        break;
    case CLEYE_MONO_RAW:
        Size = 1;
        break;
    case CLEYE_COLOR_RAW:
        Size = 4;
        break;
    case CLEYE_BAYER_RAW:
        Size = 1;
        break;
    }
    if (!FindCam()) return;
    CurrentColours = (PBYTE)malloc(W * H* Size * sizeof(byte));
}

PSEyeGetter::~PSEyeGetter()
{
    if (IsReady)
    {
        CLEyeCameraLED(Cam, false);
        CLEyeCameraStop(Cam);
        CLEyeDestroyCamera(Cam);
        free(CurrentColours);
        IsReady = false;
    }
}

bool PSEyeGetter::FindCam()
{
    if (IsReady)
    {
        CLEyeCameraStop(Cam);
        CLEyeDestroyCamera(Cam);
        IsReady = false;
    }

    if (CLEyeGetCameraCount() == 1)
    {
        guid = CLEyeGetCameraUUID(0);
        Cam = CLEyeCreateCamera(guid, Mode, Res, Rate);
        CLEyeCameraGetFrameDimensions(Cam, W, H);
        IsReady = true;
        CLEyeCameraStart(Cam);
        CLEyeCameraLED(Cam, true);
    }
    else
    {
        IsReady = false;
    }

    return IsReady;
}

bool PSEyeGetter::GetFrame()
{
    if (!IsReady)
    {
        if (!FindCam())
        {
            return false;
        }
    }
    CLEyeCameraGetFrame(Cam, CurrentColours);
    return true;
}
