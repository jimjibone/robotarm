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
        Tracker.Setup(Cam.GetCameraWidth(), Cam.GetCameraHeight(), &UpdatedLocations, (void*) this);
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

            break;
        case '2':

            break;
        case 'a':
        case 'A':
            Tracker.ShowWindow();
            break;
        case 's':
        case 'S':
            Tracker.HideWindow();
            break;
        case 'q':
        case 'Q':
            Cam.ShowImage();
            break;
        case 'w':
        case 'W':
            Cam.HideImage();
            break;
        case ' ':
            Cali.TakeCaliPoint(EyeDifferance());
            break;
        case 'c':
        case 'C':
            Cali.StartCalibration();
            break;
        case 'n':
        case 'N':

            break;
        case 'm':
        case 'M':

            break;
        }
    }

    Cam.StopCapture();
}

void EyeTracking::RunBehind()
{
    if (Cam.FindCamera())
    {
        Tracker.Setup(Cam.GetCameraWidth(), Cam.GetCameraHeight(), &UpdatedLocations, (void*) this);
        Cam.StartCapture(&UpdatedImage, (void*)this);

        Running = true;
        pthread_create(&bk_Runner, NULL, &bk_Working, (void*) this);
    }
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

            break;
        case '2':

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
    EyeTracking* This = (EyeTracking*)ptr;
    This->Img_Proc.ProcessImage(Image);
}

void EyeTracking::UpdateProcessed(IplImage* Orig, IplImage* Processed, void* ptr)
{
    EyeTracking* This = (EyeTracking*)ptr;
    This->Tracker.Track(Orig, Processed);
}

void EyeTracking::UpdatedLocations(bool Found, EyeDifferance Diff, void* ptr)
{
    EyeTracking* This = (EyeTracking*)ptr;
}
