#ifndef CALIBRATION_H
#define CALIBRATION_H

#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <pthread.h>
#include "../Variables/MultiCircleLocations.h"
#include "../Variables/GlintLocation.h"
#include "../Variables/EyeDifferance.h"
#include "../Common/EyeTimers.h"
#include "WindowsSizer.h"
#include "CalibrationScreen.h"
#include "Linear.h"
#include "Quadratic.h"

enum CaliStyle {LinearStyle, QuadraticStyle};

class Calibration
{
    public:
        Calibration();
        Calibration(CaliStyle);
        virtual ~Calibration();

        void NewValue(MultiCircleLocations, GlintLocation);

        void UpdateCaliType(CaliStyle);

        void ShowCalibrationWindow();
        void HideCalibrationWindow();

        void ShowPointWindow();
        void HidePointWindow();

        void TakeCaliPoint();

        int GetNumOfWindows();

        EyeDifferance GetCurrentDifferance();
        EyePointD GetCurrentPoint();
    protected:
    private:
        EyeTimers Timers;
        WindowsSizer Sizer;
        CalibrationScreen CaliImage;
        EyeDifferance CurDiff;
        EyePointD CurPoint;
        CaliStyle Type;

        Linear Lm, Lc;
        Quadratic Qa, Qb, Qc;

        static void* bk_Process_Thread(void*);
        bool bk_Run;

        void UpdatePointWindow();

        bool CaliWindow;
        bool PointWindow;

        EyeDifferance* CaliPoints;
        pthread_t bk_Process;

        void CaliFinished();
};

#endif // CALIBRATION_H
