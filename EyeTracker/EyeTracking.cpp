#include "EyeTracking.h"

EyeTracking::EyeTracking()
{
    Running = false;
}

EyeTracking::~EyeTracking()
{
    StopBehind();
}

void EyeTracking::Run()
{
    if (Cam.FindCamera())
    {
        Tracker.CreateTracking(Cam.GetCameraWidth(), Cam.GetCameraHeight(), &UpdatedLocations, (void*) this);
        Cam.StartCapture(&UpdatedImage, (void*)this);
    }

    bool Carry = true;
    while (Carry)
    {
        if(Cam.GetNumOfWindows() + Tracker.GetNumOfWindows() + Cali.GetNumOfWindows() == 0)
        {
            Cam.ShowImage();
        }

        switch (cvWaitKey(0))
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
            case 'q':
            Cam.ShowImage();
            break;
        case 'w':
            Cam.HideImage();
            break;
        case ' ':
            Cali.TakeCaliPoint();
            break;
        case 'c':
            Cali.ShowCalibrationWindow();
            break;
        case 'n':
            Cali.ShowPointWindow();
            break;
        case 'm':
            Cali.HidePointWindow();
            break;
        }
    }

    Cam.StopCapture();
}

void EyeTracking::RunBehind()
{
    Cam.FindCamera();
    Tracker.CreateTracking(Cam.GetCameraWidth(), Cam.GetCameraHeight(), &UpdatedLocations, (void*) this);

    Cam.StartCapture(&UpdatedImage, (void*)this);

    Running = true;
    pthread_create(&bk_Runner, NULL, &bk_Working, (void*) this);
}

void EyeTracking::StopBehind()
{
    Cam.StopCapture();
    if (Running)
    {
        Running = false;
        pthread_exit(NULL);
    }
}

void* EyeTracking::bk_Working(void* ptr)
{
    EyeTracking* This = (EyeTracking*)ptr;
    while (This->Running)
    {
        char c =  cvWaitKey(10);
        switch (c)
        {
        case 27:
            This->Running = false;
            return NULL;
        case '1':
            This->Tracker.ShowSlidersWindow();
            break;
        case '2':
            This->Tracker.HideSlidersWindow();
            break;
        case 'a':
            This->Tracker.ShowWindow();
            break;
        case 's':
            This->Tracker.HideWindow();
            break;
        case 'q':
            This->Cam.ShowImage();
            break;
        case 'w':
            This->Cam.HideImage();
            break;
        }
    }

    return NULL;
}

void EyeTracking::UpdatedImage(IplImage* Image, void* ptr)
{
    ((EyeTracking*)ptr)->Tracker.Track(Image);
}

void EyeTracking::UpdatedLocations(void* ptr)
{
    EyeTracking* This = (EyeTracking*)ptr;
    This->Cali.NewValue(This->Tracker.GetCircleLocation(), This->Tracker.GetGlintLocation());
}
