#include "../../MEngProjectFiles/PSEye/PSEyeGetter.h"

PSEyeGetter::PSEyeGetter()
{
    IsReady = false;
}

PSEyeGetter::PSEyeGetter(CLEyeCameraColorMode ColourMode, CLEyeCameraResolution Resolution, float FrameRate)
{
    IsReady = false;
    FindCam(ColourMode, Resolution, FrameRate);
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
        free(CurrentColours);
    }

    if (CLEyeGetCameraCount() == 1)
    {
        guid = CLEyeGetCameraUUID(0);
        Cam = CLEyeCreateCamera(guid, Mode, Res, Rate);
        CLEyeCameraGetFrameDimensions(Cam, W, H);
        IsReady = true;
        CLEyeCameraStart(Cam);
        CLEyeCameraLED(Cam, true);

        Stride = W * Size;
        ColourSize = H * Stride;

        CurrentColours = (PBYTE)malloc(ColourSize * sizeof(byte));
    }
    else
    {
        IsReady = false;
    }

    return IsReady;
}

bool PSEyeGetter::FindCam(CLEyeCameraColorMode ColourMode, CLEyeCameraResolution Resolution, float FrameRate)
{
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
    if (!FindCam()) return IsReady;
    return IsReady;
}

bool PSEyeGetter::GetFrame()
{
    if (!IsReady) if (!FindCam()) return false;
    CLEyeCameraGetFrame(Cam, CurrentColours);
    return true;
}

int PSEyeGetter::ImageWidth()
{
    return W;
}

int PSEyeGetter::ImageHeight()
{
    return H;
}

int PSEyeGetter::ImageDepth()
{
    return Size;
}

int PSEyeGetter::ImageStride()
{
    return Stride;
}

int PSEyeGetter::TotalPixels()
{
    return ColourSize;
}

float PSEyeGetter::FrameRate()
{
    return Rate;
}

bool PSEyeGetter::CamIsReady()
{
    return IsReady;
}
