#ifndef LINEFINDER_H
#define LINEFINDER_H

#include "Linear.h"
#include "JRPointTypes.h"
#include "../Variables/EyeDifferance.h"
#include "../Variables/EyePoint.h"
#include "../Common/WindowsSizer.h"
#include <opencv/cv.h>
#include <opencv/highgui.h>

class LineFinder
{
public:
    LineFinder();
    virtual ~LineFinder();

    void UpdateCalibration(EyeDifferance*, EyePoint*);

    EyePointD FindPoint(EyeDifferance);

    void ShowWindow();
    void HideWindow();
    void UpdateWindow(EyePointD);
    int GetNumWindows();
protected:
private:
    bool ShowWind;
    bool CaliDone;

    WindowSizer Sizer;

    PlaneCoefficients CoeY, CoeX;
};

#endif // LINEFINDER_H
