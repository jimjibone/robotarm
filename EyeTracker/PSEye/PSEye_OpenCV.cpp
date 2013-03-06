#include "../MEngProjectFiles/PSEye/PSEye_OpenCV.h"

PSEye_OpenCV::PSEye_OpenCV()
{
    Cam = PSEyeGetter(CLEYE_MONO_PROCESSED, CLEYE_VGA, 30);
    StillRunning = true;
}

PSEye_OpenCV::~PSEye_OpenCV()
{

}

bool PSEye_OpenCV::StartCapture()
{
    if (!Cam.IsReady)
    {
        if (!Cam.FindCam())
        {
            printf("Camera could not be found\n");
            return false;
        }
    }
    pthread_create(&Getter, NULL, &GetterThread, NULL);
    return true;
}

void PSEye_OpenCV::StopCapture()
{
    StillRunning = false;
    pthread_exit(NULL);
}

void PSEye_OpenCV::GetImage(IplImage Image)
{

}

void* PSEye_OpenCV::GetterThread(void* Data)
{

    return NULL;
}
