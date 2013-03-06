#include "Point.h"

EyePoint::EyePoint()
{
    //ctor
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
