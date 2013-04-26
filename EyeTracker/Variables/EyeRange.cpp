#include "EyeRange.h"

EyeRange::EyeRange()
{
    //ctor
}

EyeRange::EyeRange(unsigned char Value)
{
    int TempHigh = (int)Value + RangeArea;
    int TempLow = (int)Value - RangeArea;

    if (TempHigh > 255)
    {
        Upper = (unsigned char)255;
    }
    else
    {
        Upper = (unsigned char)TempHigh;
    }

    if (TempLow < 0)
    {
        Lower = (unsigned char)0;
    }
    else
    {
        Lower = (unsigned char)TempLow;
    }
}

EyeRange::~EyeRange()
{
    //dtor
}

bool EyeRange::CheckWithin(unsigned char Value)
{
    if (Value >= Upper | Value <= Lower) return false;
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

unsigned char EyeRange::GetUpperBoundU()
{
    return Upper;
}

unsigned char EyeRange::GetLowerBoundU()
{
    return Lower;
}
