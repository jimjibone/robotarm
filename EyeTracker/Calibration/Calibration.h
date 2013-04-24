#ifndef CALIBRATION_H
#define CALIBRATION_H

#include <opencv/cv.h>
#include <opencv/highgui.h>
#include "../Variables/EyeDifferance.h"
#include "WindowsSizer.h"
#include "CalibrationScreen.h"

typedef

class Calibration
{
    public:
        Calibration();
        virtual ~Calibration();

        void StartCalibration();
        void StopCalibration();

        void TakeCaliPoint(EyeDifferance);

        int GetNumOfWindows();

        EyeDifferance* GetCalibrationPoints();
        EyePoint* GetCalibrationLocations();
    protected:
    private:
        WindowSizer Sizer;
        CalibrationScreen CaliImage;

        bool CaliWindow;

        EyeDifferance* CaliPoints;
};

#endif // CALIBRATION_H
