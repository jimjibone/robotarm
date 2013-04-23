#include "CircleLocation.h"

CircleLocation::CircleLocation()
{
    SetRect(EyePoint(), 0, -1);
}

CircleLocation::CircleLocation(int MidX, int MidY, int InnerR)
{
    SetRect(EyePoint(MidX, MidY), InnerR, -1);
}

CircleLocation::CircleLocation(int MidX, int MidY, int InnerR, int OuterR)
{
    SetRect(EyePoint(MidX, MidY), InnerR, OuterR);
}

CircleLocation::CircleLocation(EyePoint Center, int InnerR)
{
    SetRect(Center, InnerR, -1);
}

CircleLocation::CircleLocation(EyePoint Center, int InnerR, int OuterR)
{
    SetRect(Center, InnerR, OuterR);
}

void CircleLocation::SetRect(EyePoint Point, int InnerR, int OuterR)
{
    Center = Point;
    InR = InnerR;
    InRect = EyeRectangle(Point.GetX() - InR, Point.GetY() - InR, InR * 2, InR * 2);
    OutR = OuterR;
    OutRect = EyeRectangle(Point.GetX() - OutR, Point.GetY() - OutR, OutR * 2, OutR * 2);
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

bool CircleLocation::IrisFound()
{
    return OutR != -1;
}

int CircleLocation::GetX()
{
    return Center.GetX();
}

int CircleLocation::GetY()
{
    return Center.GetY();
}

EyePoint CircleLocation::CentreLoc()
{
    return Center;
}

int CircleLocation::GetInnerRadius()
{
    return InR;
}

int CircleLocation::GetOutterRadius()
{
    return OutR;
}
