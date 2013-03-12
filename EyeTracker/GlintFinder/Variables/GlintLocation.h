#ifndef GLINTLOCATION_H
#define GLINTLOCATION_H

#include <iostream>
#include "Point.h"
#include "../../CircleFinder/Variables/Rectangle.h"
#include <opencv/cv.h>

class GlintLocation
{
    public:
        GlintLocation();
        virtual ~GlintLocation();

        void AddPoint(EyePoint Loc);

        void FindMid();
        EyePointD GetMid();

        void FindMaxRect();
        EyeRectangle GetMaxRect();
        bool WithinOld(EyePoint Loc);

        void DrawPoints(IplImage* Image);
    protected:
    private:
    int Count;
    EyePoint* Points;
    EyePointD Mid;
    EyeRectangle MaxRect;

    int Max(int Num1, int Num2);
    int Min(int Num1, int Num2);
};

#endif // GLINTLOCATION_H
