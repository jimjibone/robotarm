#include "EyeRange.h"

EyeRange::EyeRange()
{
    //ctor
}

EyeRange::EyeRange(char Value)
{
    int TempHigh = (int)Value + RangeArea;
    int TempLow = (int) Value - RangeArea;

    if (TempHigh > 255)
    {
        Upper = (char)255;
    }
    else
    {
        Upper = (char)TempHigh;
    }

    if (TempLow < 0)
    {
        Lower = (char)0;
    }
    else
    {
        Lower = (char)TempLow;
    }
}

EyeRange::~EyeRange()
{
    //dtor
}

bool EyeRange::CheckWithin(char Value)
{
    if (Value > Upper)
    {
        return false;
    }
    if (Value < Lower)
    {
        return false;
    }
    return true;
}

char EyeRange::GetUpperBound()
{
    return Upper;
}

char EyeRange::GetLowerBound()
{
    return Lower;
}
