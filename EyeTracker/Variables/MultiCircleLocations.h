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

    void AddCentre(int, int);
    void AddCentre(EyePoint);
    void AddPupilRadius(int);
    void AddIrisRadius(int);
    void AverageCircles();

    void Clear();

    EyeRectangleD PupilRectangleD();
    EyeRectangleD IrisRectangleD();

    EyeRectangle GlintSearchRectangle();
    EyeRectangle NextSearchRectangle();

    double GetIrisRadius();
    double GetPupilRadius();

    EyePointD CircleCenter();

    bool IrisFound();
protected:
private:
    EyePoint* Centers;
    int* Irises;
    int* Pupils;

    EyeRectangleD PupilRect, IrisRect;
    EyeRectangle BrightRect, NextRect;

    EyePointD Center;

    int CountCentre, CountPupil, CountIris;

    double IrisRadius, PupilRadius;

    //Constants
    static const int NewArea = 20; //How far from the old point should look next time
    static const int MaxCircles = 1000;
};

#endif // MULTICIRCLELOCATIONS_H
