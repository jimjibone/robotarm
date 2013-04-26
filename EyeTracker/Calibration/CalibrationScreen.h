#ifndef CALIBRATIONSCREEN_H
#define CALIBRATIONSCREEN_H

#include <opencv/cv.h>
#include <opencv/highgui.h>
#include "../Variables/EyePoint.h"

class CalibrationScreen
{
public:
    CalibrationScreen();
    virtual ~CalibrationScreen();
    void Setup(int, int);
    void StartCalibration(char*);

    bool NextPoint();

    int CurPoint();

    EyePoint* AllPoints();
protected:
private:
    char* WindowName;
    EyePoint* Points;

    int XStep;
    int XStart;
    int YStep;
    int YStart;
    int CurPoi;
    int Hei, Wid;

    void DrawImage();
};

#endif // CALIBRATIONSCREEN_H
