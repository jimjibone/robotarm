#include "LineFinder.h"

LineFinder::LineFinder()
{
    CurType = LinearLine;
}

LineFinder::~LineFinder()
{
    //dtor
}

void LineFinder::UpdateLineType(LineType New)
{
    CurType = New;
}

void LineFinder::UpdateCalibration(EyeDifferance* Differances, EyePoint* Points)
{
    double X1[] = { Differances[0].GetXDiff(), Differances[1].GetXDiff(), Differances[2].GetXDiff() };
    double X2[] = { Differances[3].GetXDiff(), Differances[4].GetXDiff(), Differances[5].GetXDiff() };
    double X3[] = { Differances[6].GetXDiff(), Differances[7].GetXDiff(), Differances[8].GetXDiff() };
    double Y1[] = { Differances[0].GetYDiff(), Differances[3].GetYDiff(), Differances[6].GetYDiff() };
    double Y2[] = { Differances[1].GetYDiff(), Differances[4].GetYDiff(), Differances[7].GetYDiff() };
    double Y3[] = { Differances[2].GetYDiff(), Differances[5].GetYDiff(), Differances[8].GetYDiff() };

    int PX1[] = { Points[0].GetX(), Points[1].GetX(), Points[2].GetX() };
    int PX2[] = { Points[3].GetX(), Points[4].GetX(), Points[5].GetX() };
    int PX3[] = { Points[6].GetX(), Points[7].GetX(), Points[8].GetX() };
    int PY1[] = { Points[0].GetY(), Points[3].GetY(), Points[6].GetY() };
    int PY2[] = { Points[1].GetY(), Points[4}.GetY(), Points[7].GetY() };
    int PY3[] = { Points[2].GetY(), Points[5].GetY(), Points[8].GetY() };


}

EyePointD LineFinder::FindPoint(EyeDifferance Diff)
{
    switch (CurType)
    {
    case LinearLine:
        break;
    case QuadraticLine:
        break;
    }
    return EyePointD();
}
