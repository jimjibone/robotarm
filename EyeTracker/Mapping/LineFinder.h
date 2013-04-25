#ifndef LINEFINDER_H
#define LINEFINDER_H

#include "Linear.h"
#include "Quadratic.h"
#include "../Variables/EyeDifferance.h"
#include "../Variables/EyePoint.h"

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
    int GetNumWindows();
protected:
private:
    Linear LinXm, LinXc, LinYm, LinYc;
    Quadratic QuadXa, QuadXb, QuadXc, QuadYa, QuadYb, QuadYc;

    LineType CurType;

    bool ShowWind;
};

#endif // LINEFINDER_H
