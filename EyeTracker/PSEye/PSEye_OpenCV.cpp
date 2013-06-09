#include "../MEngProjectFiles/PSEye/PSEye_OpenCV.h"

PSEye_OpenCV::PSEye_OpenCV()
{
    StillRunning = false;
    ShowWind = false;
    SaveImg = false;
    ThisTimer = EyeTimers(34L);
}

PSEye_OpenCV::~PSEye_OpenCV()
{
    StopCapture();
    if (ShowWind) HideImage();
}

bool PSEye_OpenCV::FindCamera()
{
    Cam.FindCam(CLEYE_MONO_PROCESSED, CLEYE_VGA, 30);
    return Cam.CamIsReady();
}

bool PSEye_OpenCV::StartCapture(NewImage NewImgFunc, void* Data)
{
    if (!Cam.CamIsReady()) if (!FindCamera()) return false;
    StillRunning = true;
    NewImgFunct = NewImgFunc;
    SentData = Data;
    ThisTimer.Start(&DoItFunction, (void*) this);
    return true;
}

void PSEye_OpenCV::StopCapture()
{
    if (StillRunning)
    {
        StillRunning = false;
        ThisTimer.Stop();
    }
}

void PSEye_OpenCV::DoItFunction(void* ptr)
{
    PSEye_OpenCV* This = (PSEye_OpenCV*)ptr;
    if (!This->StillRunning) return;
    if (This->Cam.GetFrame())
    {
        IplImage* CurImg = cvCreateImage(cvSize(This->Cam.ImageWidth(), This->Cam.ImageHeight()), 8, This->Cam.ImageDepth());
        for(int cnt = 0; cnt < This->Cam.TotalPixels(); cnt++)
        {
            CurImg->imageData[cnt] = (char)This->Cam.CurrentColours[cnt];
        }

        if (This->ShowWind)
        {
            cvShowImage("PSCam Image", CurImg);
        }

        if (This->SaveImg)
        {
            cvSaveImage("TestImage.png", CurImg);
            This->SaveImg = false;
            printf("Image Saved\n");
        }

        This->NewImgFunct(CurImg, This->SentData);
    }
}

void PSEye_OpenCV::ShowImage()
{
    if (!ShowWind)
    {
        cvNamedWindow("PSCam Image", CV_WINDOW_AUTOSIZE);
        ShowWind = true;
    }
}

void PSEye_OpenCV::HideImage()
{
    if (ShowWind)
    {
        ShowWind = false;
        cvDestroyWindow("PSCam Image");
        ThisTimer.Wait(100);
    }
}

int PSEye_OpenCV::GetCameraWidth()
{
    return Cam.ImageWidth();
}

int PSEye_OpenCV::GetCameraHeight()
{
    return Cam.ImageHeight();
}

int PSEye_OpenCV::GetNumOfWindows()
{
    if (ShowWind)
    {
        return 1;
    }
    else
    {
        return 0;
    }
}

void PSEye_OpenCV::SaveImage()
{
    SaveImg = true;
}
