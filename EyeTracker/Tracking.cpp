#include "Tracking.h"

Tracking::Tracking()
{
    Running = false;
    ShowWind = false;
}

Tracking::~Tracking()
{
    if (ShowWind) HideWindow();
}

void Tracking::CreateTracking(int Width, int Height)
{
    Img_proc = ImagePlaying();
    CircleFinder = HoughCircleFnder(Width, Height, true);
    GlintsFinder = GlintFinder(1);
}

void Tracking::Track(IplImage* Image)
{
    if (!Running)
    {
        Running = true;
        CurImage = Image;
        pthread_create(&bk_Process, NULL, &bk_Process_Thread, (void*) this);
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
    IplImage* CurImage = This->CurImage;
    IplImage* Image = cvCreateImage(cvGetSize(CurImage), 8, 1);
    IplImage* DispImg = cvCreateImage(cvGetSize(CurImage), 8, 3);

    if (This->ShowWind) cvCvtColor(CurImage, DispImg, CV_GRAY2BGR);

    This->Img_proc.DoAllProcesses(CurImage, Image);

    This->CircleFinder.SetOpenCVImage(Image);
    This->CircleFinder.FindCircle();

    if (This->CircleFinder.GetNumFound() == 1)
    {
        This->GlintsFinder.FindGlints(CurImage, This->CircleFinder.GetCircleLocation());

        if (This->ShowWind)
        {
            This->GlintsFinder.DrawGlints(DispImg);
            This->CircleFinder.DrawEye(DispImg);
        }
    }

    if (This->ShowWind) cvShowImage("ImageProcessing", DispImg);
    cvReleaseImage(&DispImg);

    cvReleaseImage(&Image);
    cvReleaseImage(&CurImage);

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
    ShowWind = false;
    cvDestroyWindow("ImageProcessing");
}
