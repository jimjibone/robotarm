#ifndef MULTICIRCLELOCATIONS_H
#define MULTICIRCLELOCATIONS_H

#include "CircleLocation.h"
#include "EyeRectangle.h"
#include "EyePoint.h"
#include <iostream>

class MultiCircleLocations
{
public:
    MultiCircleLocations();
    virtual ~MultiCircleLocations();

    void AddCircle(CircleLocation);
    void AverageCircles();

    EyeRectangleD PupilRectangleD();
    EyeRectangleD IrisRectangleD();

    EyeRectangle GlintSearchRectangle();
    EyeRectangle NextSearchRectangle();

    double GetIrisRadius();
    double GetPupilRadious();

    EyePointD CircleCenter();

    bool IrisFound();
protected:
private:
    CircleLocation* Circles;

    EyeRectangleD PupilRect, IrisRect;
    EyeRectangle BrightRect, NextRect;

    EyePointD Center;

    int Count;

    double IrisRadius, PupilRadius;

    //Constants
    static const int NewArea = 20; //How far from the old point should look next time
    static const int MaxCircles = 1000;
};

#endif // MULTICIRCLELOCATIONS_H
