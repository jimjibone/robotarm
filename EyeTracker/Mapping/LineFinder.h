#ifndef LINEFINDER_H
#define LINEFINDER_H

#include "Linear.h"
#include "JRPointTypes.h"
#include "../Variables/EyeDifferance.h"
#include "../Variables/EyePoint.h"
#include "../Common/WindowsSizer.h"
#include <opencv/cv.h>
#include <opencv/highgui.h>

enum LineType {LinearLine, QuadraticLine};

class LineFinder
{
public:
    LineFinder();
    virtual ~LineFinder();

    void UpdateLineType(LineType);

    void UpdateCalibration(EyeDifferance*, EyePoint*);

    EyePointD FindPoint(EyeDifferance);

    void ShowWindow();
    void HideWindow();
    void UpdateWindow(EyePointD);
    int GetNumWindows();
protected:
private:
    LineType CurType;

    bool ShowWind;

    WindowSizer Sizer;
};

#endif // LINEFINDER_H
