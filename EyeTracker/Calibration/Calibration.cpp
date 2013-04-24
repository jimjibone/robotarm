#include "Calibration.h"

Calibration::Calibration()
{
    CaliWindow = false;
    CaliImage.Setup(Sizer.GetWidth(), Sizer.GetHeight());
    CaliPoints = (EyeDifferance*)malloc(sizeof(EyeDifferance) * 9);
}

Calibration::~Calibration()
{
    StopCalibration();
}

void Calibration::StartCalibration()
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

void Calibration::StopCalibration()
{
    if (CaliWindow)
    {
        CaliWindow = false;
        cvDestroyWindow("Calibration");
    }
}

void Calibration::TakeCaliPoint(EyeDifferance Location)
{
    if (CaliWindow)
    {
        CaliPoints[CaliImage.CurPoint()] = Location;

        if (CaliImage.NextPoint()) StopCalibration();
    }
}

int Calibration::GetNumOfWindows()
{
    if (CaliWindow)
    {
        return 1;
    }
    else
    {
        return 0;
    }
}

EyeDifferance* Calibration::GetCalibrationPoints()
{
    return CaliPoints;
}

EyePoint* Calibration::GetCalibrationLocations()
{
    return CaliImage.AllPoints();
}


