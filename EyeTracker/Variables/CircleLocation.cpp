#include "CircleLocation.h"

CircleLocation::CircleLocation()
{
    SetRect(0, 0, 0, -1);
}

CircleLocation::CircleLocation(int MidX, int MidY, int InnerR)
{
    SetRect(MidX, MidY, InnerR, -1);
}

CircleLocation::CircleLocation(int MidX, int MidY, int InnerR, int OuterR)
{
    SetRect(MidX, MidY, InnerR, OuterR);
}

void CircleLocation::SetRect(int MidX, int MidY, int InnerR, int OuterR)
{
    X = MidX;
    Y = MidY;
    InR = InnerR;
    InRect = EyeRectangle(X - InR, Y - InR, InR * 2, InR * 2);
    NewRect = EyeRectangle(InRect.Left() - NewArea, InRect.Top() - NewArea, NewArea * 2 + InR * 2, NewArea * 2 + InR * 2);
    BrightRect = EyeRectangle(X - InR * 2, Y - InR * 2, InR * 4, InR * 4);
    OutR = OuterR;
    OutRect = EyeRectangle(X - OutR, Y - OutR, OutR * 2, OutR * 2);
}

CircleLocation::~CircleLocation()
{

}

EyeRectangle CircleLocation::GetInnerRectangle()
{
    return InRect;
}

EyeRectangle CircleLocation::GetOutterRectangle()
{
    return OutRect;
}

EyeRectangle CircleLocation::NextRectangle()
{
    return NewRect;
}

EyeRectangle CircleLocation::BrightRectangle()
{
    if (IrisFound())
    {
        return OutRect;
    }
    else
    {
        return BrightRect;
    }
}

bool CircleLocation::IrisFound()
{
    return OutR != -1;
}

int CircleLocation::GetX()
{
    return X;
}

int CircleLocation::GetY()
{
    return Y;
}

int CircleLocation::GetInnerRadius()
{
    return InR;
}

int CircleLocation::GetOutterRadius()
{
    return OutR;
}
