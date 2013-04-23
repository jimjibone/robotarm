#include "Calibration.h"

Calibration::Calibration()
{
    CaliWindow = false;
    PointWindow = false;
    bk_Run = false;
    CaliPoints = (EyeDifferance*)malloc(9 * sizeof(EyeDifferance));
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
        }
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


