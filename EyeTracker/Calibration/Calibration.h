#ifndef CALIBRATION_H
#define CALIBRATION_H

#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <pthread.h>
#include "../Variables/MultiCircleLocations.h"
#include "../Variables/GlintLocation.h"
#include "../Common/EyeTimers.h"
#include "WindowsSizer.h"
#include "CalibrationScreen.h"

class Calibration
{
    public:
        Calibration();
        virtual ~Calibration();

        void NewValue(MultiCircleLocations, GlintLocation);

        void ShowCalibrationWindow();
        void HideCalibrationWindow();

        void ShowPointWindow();
        void HidePointWindow();

        void TakeCaliPoint();

        int GetNumOfWindows();

        EyePointD GetCurrentPoint();
    protected:
    private:
        EyeTimers Timers;
        WindowsSizer Sizer;
        CalibrationScreen CaliImage;

        void UpdatePointWindow();

        bool CaliWindow;
        bool PointWindow;

        int NumOfCalis;

        EyePointD* CalibrationPoints;
};

#endif // CALIBRATION_H
