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

void Tracking::CreateTracking(int Width, int Height, PosUpdate Func, void* Data)
{
    Img_proc = ImagePlaying(100, 255);
    CircleFinder = HoughCircleFnder(Width, Height, false);
    GlintsFinder = GlintFinder(1);
    UpdateFuncs = Func;
    SentData = Data;
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

GlintLocation Tracking::GetGlintLocation()
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

MultiCircleLocations Tracking::GetCircleLocation()
{
    if (CircleFinder.GetNumFound() == 1)
    {
        return CircleFinder.GetCircleLocation();
    }
    else
    {
        return MultiCircleLocations();
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

        This->UpdateFuncs(This->SentData);

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
        Timers.Wait(100);
    }
}

void Tracking::ShowSlidersWindow()
{
    if (!ShowTrackWind)
    {
        ShowTrackWind = true;
        cvNamedWindow("Settings", CV_WINDOW_AUTOSIZE);
        int Min = Img_proc.GetNum1();
        int Max = Img_proc.GetNum2();
        cvCreateTrackbar2("Min", "Settings", &Min, 255, &MinChange, (void*) this);
        cvCreateTrackbar2("Max", "Settings", &Max, 255, &MaxChange, (void*) this);
    }
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

int Tracking::GetNumOfWindows()
{
    int Num = 0;

    if (ShowWind) Num++;
    if (ShowTrackWind) Num++;

    return Num;
}
