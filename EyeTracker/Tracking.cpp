#include "Tracking.h"

Tracking::Tracking()
{
    Runing = false;
}

Tracking::Tracking(PSEye_OpenCV* Input)
{
    Runing = false;
    InputsFrom = Input;
    Img_proc = ImagePlaying();
    CircleFinder = HoughCircleFnder(640, 480, true);
    GlintsFinder = GlintFinder(1);
}

Tracking::~Tracking()
{
    StopTracking();
}

void Tracking::StartTracking()
{
    Runing = true;
    pthread_create(&bk_Process, NULL, &bk_Process_Thread, (void*) this);
}

void Tracking::StopTracking()
{
    bool Temp = Runing;
    Runing = false;
    if (Temp) pthread_exit(NULL);
}

GlintLocation Tracking::GetCurPoint()
{
    if (CircleFinder.GetNumFound() == 1)
    {
        return GlintsFinder.GetGlintLocation(0);
    }
    else
    {
        return GlintLocation();
    }
}

CircleLocation Tracking::GetCurEyePoint()
{
    if (CircleFinder.GetNumFound() == 1)
    {
        return CircleFinder.GetCircleLocation();
    }
    else
    {
        return CircleLocation();
    }
}

void* Tracking::bk_Process_Thread(void* Input)
{
    Tracking* This = (Tracking*)Input;
    IplImage* Image = cvCreateImage(cvSize(640, 480), 8, 1);

    while (This->Runing)
    {
        IplImage* CurImg = This->InputsFrom->GetImage();
        This->Img_proc.DoAllProcesses(CurImg, Image);
        This->CircleFinder.SetOpenCVImage(Image);
        This->CircleFinder.FindCircle();
        if (This->CircleFinder.GetNumFound() == 1)
        {
            This->GlintsFinder.FindGlints(CurImg, This->CircleFinder.GetCircleLocation());
        }
    }

    cvReleaseImage(&Image);
    return NULL;
}
