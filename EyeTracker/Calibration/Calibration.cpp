#include "Calibration.h"

Calibration::Calibration()
{
    CaliWindow = false;
    PointWindow = false;
    bk_Run = false;
    CaliPoints = (EyeDifferance*)malloc(9 * sizeof(EyeDifferance));
    Type = LinearStyle;
}

Calibration::Calibration(CaliStyle Style)
{
    Type = Style;
    CaliWindow = false;
    PointWindow = false;
    bk_Run = false;
    CaliPoints = (EyeDifferance*)malloc(9 * sizeof(EyeDifferance));
}

void Calibration::UpdateCaliType(CaliStyle Style)
{
    Type = Style;
}

Calibration::~Calibration()
{
    HideCalibrationWindow();
    HidePointWindow();
}

void Calibration::NewValue(MultiCircleLocations Circle, GlintLocation Glint)
{
    if (!bk_Run)
    {
        CurDiff = EyeDifferance(Circle.CircleCenter(), Glint.GetMid());
        if (!CaliWindow)
        {
            bk_Run = true;
            pthread_create(&bk_Process, NULL, &bk_Process_Thread, (void*) this);
        }
    }
}

void* Calibration::bk_Process_Thread(void* Input)
{
    Calibration* This = (Calibration*) Input;
    return NULL;
}

EyePointD Calibration::GetCurrentPoint()
{
    return CurPoint;
}

EyeDifferance Calibration::GetCurrentDifferance()
{
    return CurDiff;
}

void Calibration::ShowCalibrationWindow()
{
    if (!CaliWindow)
    {
        cvNamedWindow("Calibration", CV_WINDOW_FULLSCREEN);
        cvMoveWindow("Calibration", 0, 0);
        Sizer.SetWindowToFullScreen("Calibration");

        CaliImage.StartCalibration("Calibration");

        CaliWindow = true;
    }
}

void Calibration::HideCalibrationWindow()
{
    if (CaliWindow)
    {
        CaliWindow = false;
        cvDestroyWindow("Calibration");
        Timers.Wait(100);
    }
}

void Calibration::TakeCaliPoint()
{
    if (CaliWindow)
    {
        CaliPoints[CaliImage.CurPoint()] = GetCurrentDifferance();

        if (CaliImage.NextPoint())
        {
            HideCalibrationWindow();
            CaliFinished();
        }
    }
}

void Calibration::CaliFinished()
{
    double X1[] = {CaliPoints[0].GetXDiff(), CaliPoints[1].GetXDiff(), CaliPoints[2].GetXDiff()};
    double X2[] = {CaliPoints[3].GetXDiff(), CaliPoints[4].GetXDiff(), CaliPoints[5].GetXDiff()};
    double X3[] = {CaliPoints[6].GetXDiff(), CaliPoints[7].GetXDiff(), CaliPoints[8].GetXDiff()};
    double Y1[] = {CaliPoints[0].GetYDiff(), CaliPoints[3].GetYDiff(), CaliPoints[6].GetYDiff()};
    double Y2[] = {CaliPoints[1].GetYDiff(), CaliPoints[4].GetYDiff(), CaliPoints[7].GetYDiff()};
    double Y3[] = {CaliPoints[2].GetYDiff(), CaliPoints[5].GetYDiff(), CaliPoints[8].GetYDiff()};

    Linear L1, L2, L3;
    Quadratic Q1, Q2, Q3;

    EyePoint[] AllPoints = CaliImage.AllPoints();

    double SX1[] = {(double)AllPoints[0].GetX(), (double)AllPoints[1].GetX(), (double)AllPoints[2].GetX()};
    double SX2[] = {(double)AllPoints[3].GetX(), (double)AllPoints[4].GetX(), (double)AllPoints[5].GetX()};
    double SX3[] = {(double)AllPoints[6].GetX(), (double)AllPoints[7].GetX(), (double)AllPoints[8].GetX()};
    double SY1[] = {(double)AllPoints[0].GetY(), (double)AllPoints[3].GetY(), (double)AllPoints[6].GetY()};
    double SY2[] = {(double)AllPoints[1].GetY(), (double)AllPoints[4].GetY(), (double)AllPoints[7].GetY()};
    double SY3[] = {(double)AllPoints[2].GetY(), (double)AllPoints[5].GetY(), (double)AllPoints[8].GetY()};

    switch(Type)
    {
    case LinearStyle:
        L1.FindLine(X1, SY1, 3);
        L2.FindLine(X2, SY2, 3);
        L3.FindLine(X3, SY3, 3);

        double ms[] = {L1.Getm(), L2.Getm(), L3.Getm()};
        double cs[] = {L1.Getc(), L2.Getc(), L3.Getc()};

        Lm.FindLine(SX1, ms, 3);
        Lm.FindLine(SX1, cs, 3);
        break;
    case QuadraticStyle:
        Q1.FindLine(SX1, Y1, 3);
        Q1.FindLine(SX2, Y1, 3);
        Q1.FindLine(SX3, Y1, 3);
        break;
    }
}

void Calibration::ShowPointWindow()
{
    if (!PointWindow)
    {
        cvNamedWindow("CurPoint", CV_WINDOW_FULLSCREEN);
        cvMoveWindow("CurPoint", 0, 0);
        Sizer.SetWindowToFullScreen("CurPoint");
        PointWindow = true;
    }
}

void Calibration::HidePointWindow()
{
    if (PointWindow)
    {
        PointWindow = false;
        cvDestroyWindow("CurPoint");
        Timers.Wait(100);
    }
}

void Calibration::UpdatePointWindow()
{
    if (PointWindow)
    {
        IplImage* Image = cvCreateImage(cvSize(Sizer.GetWidth(), Sizer.GetHeight()), 8, 3);
        EyePointD Cur = GetCurrentPoint();
        cvCircle(Image, cvPoint(Cur.GetXint(), Cur.GetYint()), 10, CV_RGB(255, 255, 255), 10);
        cvShowImage("CurPoint", Image);
        cvReleaseImage(&Image);
    }
}

int Calibration::GetNumOfWindows()
{
    int Num = 0;

    if (CaliWindow) Num++;
    if (PointWindow) Num++;

    return Num;
}


