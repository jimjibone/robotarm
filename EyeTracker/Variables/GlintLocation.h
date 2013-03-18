#ifndef GLINTLOCATION_H
#define GLINTLOCATION_H

#include <iostream>
#include "EyePoint.h"
#include "EyeRectangle.h"
#include <opencv/cv.h>

using namespace std;

class GlintLocation
{
    public:
        GlintLocation();
        virtual ~GlintLocation();

        void AddPoint(EyePoint);

        void FindMid();
        EyePointD GetMid();

        void FindMaxRect();
        EyeRectangle GetMaxRect();
        bool WithinOld(EyePoint);

        void DrawPoints(IplImage*);
    protected:
    private:
    int Count;
    EyePoint* Points;
    EyePointD Mid;
    EyeRectangle MaxRect;

    int Max(int Num1, int Num2);
    int Min(int Num1, int Num2);

    static const int MaxNumOfPoints = 2000;
};

#endif // GLINTLOCATION_H
