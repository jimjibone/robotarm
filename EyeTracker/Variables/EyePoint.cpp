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

AvEyePointD::AvEyePointD()
{
    NumAt = 0;
    MaxNum = 5;
    First = true;
    Points = (EyePointD*)malloc(sizeof(EyePointD) * MaxNum);
}

AvEyePointD::AvEyePointD(int ToExpect)
{
    NumAt = 0;
    MaxNum = ToExpect;
    First = true;
    Points = (EyePointD*)malloc(sizeof(EyePointD) * MaxNum);
}

AvEyePointD::~AvEyePointD()
{

}

void AvEyePointD::Reset()
{
    NumAt = 0;
    First = true;
}

void AvEyePointD::AddPoint(EyePointD NewPoint)
{
    Points[NumAt] = NewPoint;
    NumAt++;
    if (NumAt < MaxNum)
    {
        NumAt = 0;
        First = false;
    }
}

EyePointD AvEyePointD::GetCurAverage()
{
    if (First)
    {
        double Xs = 0, Ys = 0;
        for (int cnt = 0; cnt <= NumAt; cnt++)
        {
            Xs += Points[cnt].GetX();
            Ys += Points[cnt].GetY();
        }
        Xs /= (NumAt + 1);
        Ys /= (NumAt + 1);
        return EyePointD(Xs, Ys);
    }
    else
    {
        double Xs = 0, Ys = 0;
        for (int cnt = 0; cnt <= MaxNum; cnt++)
        {
            Xs += Points[cnt].GetX();
            Ys += Points[cnt].GetY();
        }
        Xs /= (NumAt + 1);
        Ys /= (NumAt + 1);
        return EyePointD(Xs, Ys);
    }
}
