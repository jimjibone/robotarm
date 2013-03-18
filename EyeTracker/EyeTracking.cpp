#include "EyeTracking.h"

EyeTracking::EyeTracking()
{
    //ctor
}

EyeTracking::~EyeTracking()
{
    //dtor
}

void EyeTracking::Run()
{
    Cam.FindCamera();
    Tracker.CreateTracking(Cam.GetCameraWidth(), Cam.GetCameraHeight(), &UpdatedLocations, (void*) this);

    Cam.StartCapture(&UpdatedImage, (void*)this);

    bool Carry = true;
    while (Carry)
    {
        char c =  cvWaitKey(0);
        switch (c)
        {
        case 27:
            Carry = false;
            break;
        case '1':
            Tracker.ShowSlidersWindow();
            break;
        case '2':
            Tracker.HideSlidersWindow();
            break;
        case 'a':
            Tracker.ShowWindow();
            break;
        case 's':
            Tracker.HideWindow();
            break;
        }
    }

    Cam.StopCapture();
}

void* EyeTracking::bk_Working(void*)
{
    return NULL;
}

void EyeTracking::UpdatedImage(IplImage* Image, void* ptr)
{
    ((EyeTracking*)ptr)->Tracker.Track(Image);
}

void EyeTracking::UpdatedLocations(void* ptr)
{
    EyeTracking* This = (EyeTracking*)ptr;
    CircleLocation Circle = This->Tracker.GetCircleLocation();
    GlintLocation Glint = This->Tracker.GetGlintLocation();
    printf("Circle: (%d, %d), Glint: (%f, %f)\n", Circle.GetX(), Circle.GetY(), Glint.GetMid().GetX(), Glint.GetMid().GetY());
}
