#include "EyeDifferance.h"

EyeDifferance::EyeDifferance()
{
    DiffX = 0;
    DiffY = 0;
    intDiffX = 0;
    intDiffY = 0;
}

EyeDifferance::EyeDifferance(EyePoint One, EyePoint Two)
{
    DiffX = One.GetX() - Two.GetX();
    DiffY = One.GetY() - Two.GetY();
    intDiffX = (int)DiffX;
    intDiffY = (int)DiffY;
}

EyeDifferance::EyeDifferance(EyePoint One, EyePointD Two)
{
    DiffX = One.GetX() - Two.GetX();
    DiffY = One.GetY() - Two.GetY();
    intDiffX = (int)DiffX;
    intDiffY = (int)DiffY;
}

EyeDifferance::EyeDifferance(EyePointD One, EyePoint Two)
{
    DiffX = One.GetX() - Two.GetX();
    DiffY = One.GetY() - Two.GetY();
    intDiffX = (int)DiffX;
    intDiffY = (int)DiffY;
}

EyeDifferance::EyeDifferance(EyePointD One, EyePointD Two)
{
    DiffX = One.GetX() - Two.GetX();
    DiffY = One.GetY() - Two.GetY();
    intDiffX = (int)DiffX;
    intDiffY = (int)DiffY;
}

EyeDifferance::~EyeDifferance()
{
    //dtor
}

double EyeDifferance::GetXDiff()
{
    return DiffX;
}

double EyeDifferance::GetYDiff()
{
    return DiffY;
}

int EyeDifferance::GetXDiffint()
{
    return intDiffX;
}

int EyeDifferance::GetYDiffint()
{
    return intDiffY;
}

