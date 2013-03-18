#include "EyePoint.h"

EyePoint::EyePoint()
{
    EyePoint::X = 0;
    EyePoint::Y = 0;
}

EyePoint::EyePoint(int X, int Y)
{
    EyePoint::X = X;
    EyePoint::Y = Y;
}

EyePoint::~EyePoint()
{
    //dtor
}

int EyePoint::GetY()
{
    return Y;
}

int EyePoint::GetX()
{
    return X;
}

EyePointD::EyePointD()
{
    EyePointD::X = 0;
    EyePointD::Y = 0;
}

EyePointD::EyePointD(double X, double Y)
{
    EyePointD::X = X;
    EyePointD::Y = Y;
}

EyePointD::~EyePointD()
{
    //dtor
}

double EyePointD::GetY()
{
    return Y;
}

double EyePointD::GetX()
{
    return X;
}

int EyePointD::GetYint()
{
    return (int)Y;
}

int EyePointD::GetXint()
{
    return (int)X;
}
