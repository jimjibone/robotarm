#include "Tracking.h"

Tracking::Tracking()
{
    Running = false;
    ShowWind = false;
    open = false;
}

Tracking::~Tracking()
{
    HideWindow();
    if (open) myFile.close();
}

void Tracking::Setup(int Width, int Height, PosUpdate Func, void* Data)
{
    CircleFinder = HoughCircleFnder(Width, Height, false);
    UpdateFuncs = Func;
    SentData = Data;

    myFile.open("TestData.txt", ios::out);
    open = true;
    if (open)
    {
        myFile << "Circle\t\t\t\t\tGlint\n";
        myFile << "Current X\tCurrent Y\tPupilRadius\tAverage X\tAverage Y\tCurrent X\tCurrent Y\tAverage X\t Average Y\n";
    }
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
    IplImage* DispImg;
    if (This->ShowWind) DispImg = cvCreateImage(cvGetSize(This->CurImage), 8, 3);

    if (DispImg->width != 0 && This->ShowWind) cvCvtColor(This->OrigImage, DispImg, CV_GRAY2BGR);

    This->CircleFinder.SetOpenCVImage(This->CurImage);
    This->CircleFinder.FindCircle();

    if (This->CircleFinder.Found())
    {
        This->GlintsFinder.FindGlints(This->OrigImage, This->CircleFinder.GetCircleLocation().GlintSearchRectangle());

        This->CircleAv.AddPoint(This->CircleFinder.GetCircleLocation().CircleCenter());
        This->GlintAv.AddPoint(This->GlintsFinder.GetGlintLocation().GetMid());

        if (DispImg->width != 0 && This->ShowWind)
        {
            This->GlintsFinder.DrawGlints(DispImg);
            This->CircleFinder.DrawEye(DispImg);
        }
    }

    EyePointD CirAv = This->CircleAv.GetCurAverage();
    EyePointD GliAv = This->GlintAv.GetCurAverage();

    if (This->open)
    {
        if (This->CircleFinder.Found())
        {
            This->myFile << This->CircleFinder.GetCircleLocation().CircleCenter().GetX() << "\t" <<
            This->CircleFinder.GetCircleLocation().CircleCenter().GetY() << "\t" <<
            This->CircleFinder.GetCircleLocation().GetPupilRadius() << "\t" <<
            CirAv.GetX() << "\t" <<
            CirAv.GetY() << "\t" <<
            This->GlintsFinder.GetGlintLocation().GetMid().GetX() << "\t" <<
            This->GlintsFinder.GetGlintLocation().GetMid().GetY() << "\t" <<
            CirAv.GetX() << "\t" <<
            CirAv.GetY() << "\n";
        }
        else
        {
            This->myFile << "\n";
        }
    }

    This->UpdateFuncs(This->CircleFinder.Found(),
                      EyeDifferance(CirAv, GliAv),
                      This->SentData);

    if (DispImg->width != 0 && This->ShowWind)  cvShowImage("ImageProcessing", DispImg);

    if (DispImg->width != 0) cvReleaseImage(&DispImg);
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
