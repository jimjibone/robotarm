#include "Tracking.h"

Tracking::Tracking()
{
    Running = false;
    ShowWind = false;
    ShowTrackWind = false;
}

Tracking::~Tracking()
{
    HideWindow();
    HideSlidersWindow();
}

void Tracking::CreateTracking(int Width, int Height, PosUpdate Func)
{
    Img_proc = ImagePlaying(125, 175);
    CircleFinder = HoughCircleFnder(Width, Height, true);
    GlintsFinder = GlintFinder(1);
    UpdateFuncs = Func;
}

void Tracking::Track(IplImage* Image)
{
    if (!Running)
    {
        Timers.Wait(1);
        Running = true;
        CurImage = Image;
        pthread_create(&bk_Process, NULL, &bk_Process_Thread, (void*) this);
    }
    else
    {
        cvReleaseImage(&Image);
    }
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
    Tracking* This = (Tracking*) Input;
    IplImage* Image = cvCreateImage(cvGetSize(This->CurImage), 8, 1);
    IplImage* DispImg = cvCreateImage(cvGetSize(This->CurImage), 8, 3);

    if (This->ShowWind) cvCvtColor(This->CurImage, DispImg, CV_GRAY2BGR);

    This->Img_proc.DoAllProcesses(This->CurImage, Image);

    This->CircleFinder.SetOpenCVImage(Image);
    This->CircleFinder.FindCircle();

    if (This->CircleFinder.GetNumFound() == 1)
    {
        This->GlintsFinder.FindGlints(This->CurImage, This->CircleFinder.GetCircleLocation());

        This->UpdateFuncs(This->CircleFinder.GetCircleLocation(), This->GlintsFinder.GetGlintLocation(0));

        if (This->ShowWind)
        {
            This->GlintsFinder.DrawGlints(DispImg);
            This->CircleFinder.DrawEye(DispImg);
        }
    }

    if (This->ShowWind) cvShowImage("ImageProcessing", DispImg);
    if (This->ShowTrackWind) cvShowImage("Settings", Image);

    cvReleaseImage(&DispImg);
    cvReleaseImage(&Image);
    cvReleaseImage(&This->CurImage);

    This->Running = false;
    return NULL;
}

void Tracking::ShowWindow()
{
    ShowWind = true;
    cvNamedWindow("ImageProcessing", CV_WINDOW_AUTOSIZE);
}

void Tracking::HideWindow()
{
    if (ShowWind)
    {
        ShowWind = false;
        cvDestroyWindow("ImageProcessing");
        Timers.Wait(100);
    }
}

void Tracking::ShowSlidersWindow()
{
    ShowTrackWind = true;
    cvNamedWindow("Settings", CV_WINDOW_AUTOSIZE);
    int Min = Img_proc.GetNum1();
    int Max = Img_proc.GetNum2();
    cvCreateTrackbar2("Min", "Settings", &Min, 255, &MinChange, (void*) this);
    cvCreateTrackbar2("Max", "Settings", &Max, 255, &MaxChange, (void*) this);
}

void Tracking::HideSlidersWindow()
{
    if (ShowTrackWind)
    {
        ShowTrackWind = false;
        cvDestroyWindow("Settings");
		Timers.Wait(100);
    }
}

void Tracking::MinChange(int Num, void* ptr)
{
    ((Tracking*)ptr)->Img_proc.SetNum1(Num);
}

void Tracking::MaxChange(int Num, void* ptr)
{
    ((Tracking*)ptr)->Img_proc.SetNum2(Num);
}
