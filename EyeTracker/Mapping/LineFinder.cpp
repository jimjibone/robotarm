#include "LineFinder.h"

LineFinder::LineFinder()
{
    CurType = LinearLine;
    ShowWind = false;
}

LineFinder::~LineFinder()
{
    //dtor
}

void LineFinder::UpdateLineType(LineType New)
{
    CurType = New;
}

void LineFinder::UpdateCalibration(EyeDifferance* Differances, EyePoint* Points)
{

}

EyePointD LineFinder::FindPoint(EyeDifferance Diff)
{
    switch (CurType)
    {
    case LinearLine:
        break;
    case QuadraticLine:
        break;
    }
    return EyePointD();
}

void LineFinder::ShowWindow()
{
    if (!ShowWind)
    {
        cvNamedWindow("Current Point", CV_WINDOW_FULLSCREEN);
        cvMoveWindow("Current Point", 0, 0);
        Sizer.SetWindowToFullScreen("Current Point");
        ShowWind = true;
    }
}

void LineFinder::HideWindow()
{
    if (ShowWind)
    {
        ShowWind = false;
        cvDestroyWindow("Current Point");
    }
}

void LineFinder::UpdateWindow(EyePointD Point)
{
    if (ShowWind)
    {
        IplImage* Image = cvCreateImage(cvSize(Sizer.GetWidth(), Sizer.GetHeight()), 8, 3);
        cvCircle(Image, cvPoint(Point.GetXint(), Point.GetYint()), 15, CV_RGB(255, 255, 0), 0);
        cvShowImage("Current Point", Image);
        cvReleaseImage(&Image);
    }
}

int LineFinder::GetNumWindows()
{
    if (ShowWind) return 1;
    return 0;
}
