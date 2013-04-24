#ifndef CALIBRATIONSCREEN_H
#define CALIBRATIONSCREEN_H

#include "WindowsSizer.h"
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include "../Variables/EyePoint.h"

class CalibrationScreen
{
    public:
        CalibrationScreen();
        virtual ~CalibrationScreen();

        void StartCalibration(char*);

        bool NextPoint();

        int CurPoint();

        EyePoint[] AllPoints();
    protected:
    private:
        WindowsSizer Sizer;
        char* WindowName;

        int XStep;
        int XStart;
        int YStep;
        int YStart;

        void AddX();

        void DrawImage();

        int CurX;
        int CurY;
};

#endif // CALIBRATIONSCREEN_H
