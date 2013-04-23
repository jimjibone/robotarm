#include "Calibration.h"

Calibration::Calibration()
{
    CaliWindow = false;
    PointWindow = false;
    CalibrationPoints = (EyePointD*)malloc(9 * sizeof(EyePointD));
}

Calibration::~Calibration()
{
    HideCalibrationWindow();
    HidePointWindow();
}

void Calibration::NewValue(MultiCircleLocations, GlintLocation)
{

}

EyePointD Calibration::GetCurrentPoint()
{
    return EyePointD();
}

void Calibration::ShowCalibrationWindow()
{
    if (!CaliWindow)
    {
        NumOfCalis = 0;
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
        CalibrationPoints[CaliImage.CurPoint()] = GetCurrentPoint();

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


