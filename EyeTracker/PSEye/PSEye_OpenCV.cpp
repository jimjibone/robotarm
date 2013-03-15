#include "../MEngProjectFiles/PSEye/PSEye_OpenCV.h"

PSEye_OpenCV::PSEye_OpenCV()
{
    StillRunning = false;
    ShowWind = false;
    Ready = false;
    InLineRunning = false;
    CurImage = -1;
    ThisTimer = EyeTimers(34L);
}

PSEye_OpenCV::~PSEye_OpenCV()
{
    StopCapture();
    cvReleaseImage(&Img_1);
    cvReleaseImage(&Img_2);
    cvReleaseImage(&Img_3);
}

bool PSEye_OpenCV::FindCamera()
{
    Cam = PSEyeGetter(CLEYE_MONO_PROCESSED, CLEYE_VGA, 30);
    if (!Cam.IsReady) return false;
    Img_1 = cvCreateImage(cvSize(Cam.W, Cam.H), 8, 1);
    Img_2 = cvCreateImage(cvSize(Cam.W, Cam.H), 8, 1);
    Img_3 = cvCreateImage(cvSize(Cam.W, Cam.H), 8, 1);
    Ready = true;
    return true;
}

bool PSEye_OpenCV::StartCapture()
{
    if (!Ready) if (!FindCamera()) return false;
    ThisTimer.Start(&DoItFunction, (void*) this);
    StillRunning = true;
    return true;
}

void PSEye_OpenCV::StopCapture()
{
    ThisTimer.Stop();
    StillRunning = false;
}

IplImage* PSEye_OpenCV::GetImage()
{
    switch(CurImage)
    {
    case 0:
        return Img_1;
    case 1:
        return Img_2;
    case 2:
        return Img_3;
    }
    return NULL;
}

void PSEye_OpenCV::DoItFunction(void* ptr)
{
    PSEye_OpenCV* This = (PSEye_OpenCV*)ptr;
    if (!This->StillRunning) return;
    if (This->Cam.GetFrame())
    {
        This->CurImage++;
        if (This->CurImage >= 3)
        {
            This->CurImage = 0;
        }
        IplImage* CurImg = This->GetImage();
        int To = This->Cam.W * This->Cam.H * This->Cam.Size;
        for(int cnt = 0; cnt < To; cnt++)
        {
            CurImg->imageData[cnt] = (char)This->Cam.CurrentColours[cnt];
        }

        if (This->ShowWind)
        {
            cvShowImage("PSCam Image", CurImg);
        }
    }
}

void PSEye_OpenCV::ShowImage()
{
    cvNamedWindow("PSCam Image", CV_WINDOW_AUTOSIZE);
    ShowWind = true;

}

void PSEye_OpenCV::HideImage()
{
    ShowWind = false;
    cvDestroyWindow("PSCam Image");
}

void PSEye_OpenCV::StartOtherCapture()
{
    pthread_create(&Line_Thread, NULL, &InLineFunnction, (void*) this);
    InLineRunning = true;
}

void PSEye_OpenCV::StopOtherCapture()
{
    bool Run = InLineRunning;
    InLineRunning = false;
    if (Run) pthread_exit(NULL);
}

void* PSEye_OpenCV::InLineFunnction(void* Input)
{
    PSEye_OpenCV* This = (PSEye_OpenCV*)Input;

    PSEyeGetter Cam = PSEyeGetter(CLEYE_MONO_PROCESSED, CLEYE_VGA, 30);
    if (!Cam.IsReady) return NULL;
    This->Img_1 = cvCreateImage(cvSize(Cam.W, Cam.H), 8, 1);
    This->Img_2 = cvCreateImage(cvSize(Cam.W, Cam.H), 8, 1);
    This->Img_3 = cvCreateImage(cvSize(Cam.W, Cam.H), 8, 1);

    while (This->InLineRunning)
    {
        This->ThisTimer.WaitTillNext();
        if (Cam.GetFrame())
        {
            This->CurImage++;
            if (This->CurImage >= 3)
            {
                This->CurImage = 0;
            }
            IplImage* CurImg = This->GetImage();
            int To = This->Cam.W * This->Cam.H * This->Cam.Size;

            for(int cnt = 0; cnt < To; cnt++)
            {
                CurImg->imageData[cnt] = (char)This->Cam.CurrentColours[cnt];
            }

            if (This->ShowWind)
            {
                cvShowImage("PSCam Image", CurImg);
            }
        }
    }

    return NULL;
}
