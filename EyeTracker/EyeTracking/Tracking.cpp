#include "Tracking.h"

Tracking::Tracking()
{
    Running = false;
    ShowWind = false;
}

Tracking::~Tracking()
{
    HideWindow();
}

void Tracking::Setup(int Width, int Height, PosUpdate Func, void* Data)
{
    CircleFinder = HoughCircleFnder(Width, Height, false);
    UpdateFuncs = Func;
    SentData = Data;
}

void Tracking::Track(IplImage* Orig, IplImage* Image)
{
    if (!Running)
    {
        Running = true;
        CurImage = Image;
        OrigImage = Orig;
        pthread_create(&bk_Process, NULL, &bk_Process_Thread, (void*) this);
    }
    else
    {
        cvReleaseImage(&Image);
        cvReleaseImage(&Orig);
    }
}

EyePointD Tracking::GetGlintLocation()
{
    return GlintsFinder.GetGlintLocation().GetMid();
}

EyePointD Tracking::GetCircleLocation()
{
    return CircleFinder.GetCircleLocation().CircleCenter();
}

void* Tracking::bk_Process_Thread(void* Input)
{
    Tracking* This = (Tracking*) Input;
    IplImage* DispImg = cvCreateImage(cvGetSize(This->CurImage), 8, 3);

    if (This->ShowWind) cvCvtColor(This->OrigImage, DispImg, CV_GRAY2BGR);

    This->CircleFinder.SetOpenCVImage(This->CurImage);
    This->CircleFinder.FindCircle();

    if (This->CircleFinder.Found())
    {
        This->GlintsFinder.FindGlints(This->OrigImage, This->CircleFinder.GetCircleLocation().GlintSearchRectangle());

        if (This->ShowWind)
        {
            This->GlintsFinder.DrawGlints(DispImg);
            This->CircleFinder.DrawEye(DispImg);
        }
    }

    This->UpdateFuncs(This->CircleFinder.Found(),
                      EyeDifferance(This->CircleFinder.GetCircleLocation().CircleCenter(), This->GlintsFinder.GetGlintLocation().GetMid()),
                      This->SentData);

    if (This->ShowWind) cvShowImage("ImageProcessing", DispImg);

    cvReleaseImage(&DispImg);
    cvReleaseImage(&This->CurImage);

    This->Running = false;
    return NULL;
}

void Tracking::ShowWindow()
{
    if (!ShowWind)
    {
        ShowWind = true;
        cvNamedWindow("ImageProcessing", CV_WINDOW_AUTOSIZE);
    }
}

void Tracking::HideWindow()
{
    if (ShowWind)
    {
        ShowWind = false;
        cvDestroyWindow("ImageProcessing");
    }
}


int Tracking::GetNumOfWindows()
{
    int Num = 0;

    if (ShowWind) Num++;

    return Num;
}
