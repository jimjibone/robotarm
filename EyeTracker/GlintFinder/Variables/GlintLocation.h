#ifndef GLINTLOCATION_H
#define GLINTLOCATION_H

#include <iostream>
#include "Point.h"
#include "../../CircleFinder/Variables/Rectangle.h"

class GlintLocation
{
    public:
        GlintLocation();
        virtual ~GlintLocation();

        void AddPoint(EyePoint Loc);

        void FindMid();
        EyePoint GetMid();

        void FindMaxRect();
        bool WithinOld(EyePoint Loc);
    protected:
    private:
    int Count;
    EyePoint* Points;
    EyePoint Mid;
    EyeRectangle MaxRect;

    int Max(int Num1, int Num2);
    int Min(int Num1, int Num2);
};

#endif // GLINTLOCATION_H
