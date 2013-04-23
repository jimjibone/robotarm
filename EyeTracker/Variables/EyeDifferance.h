#ifndef EYEDIFFERANCE_H
#define EYEDIFFERANCE_H

#include "EyePoint.h"

class EyeDifferance
{
    public:
        EyeDifferance();
        EyeDifferance(EyePoint, EyePoint);
        EyeDifferance(EyePoint, EyePointD);
        EyeDifferance(EyePointD, EyePoint);
        EyeDifferance(EyePointD, EyePointD);

        double GetXDiff();
        double GetYDiff();
        int GetXDiffint();
        int GetYDiffint();

        virtual ~EyeDifferance();
    protected:
    private:
    double DiffX, DiffY;
    int intDiffX, intDiffY;
};

#endif // EYEDIFFERANCE_H
