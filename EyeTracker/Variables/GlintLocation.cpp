#include "GlintLocation.h"

GlintLocation::GlintLocation()
{
    Points = (EyePoint*)malloc(MaxNumOfPoints * sizeof(EyePoint));
    Count = 0;
}

GlintLocation::~GlintLocation()
{

}

void GlintLocation::AddPoint(EyePoint Loc)
{
    if (Count < MaxNumOfPoints - 1)
    {
        Points[Count] = Loc;
        Count++;
    }
}

void GlintLocation::Clear()
{
    Count = 0;
}

void GlintLocation::FindMid()
{
    double X = 0, Y = 0;
    for (int cnt = 0; cnt < Count; cnt++)
    {
        X += (double)Points[cnt].GetX();
        Y += (double)Points[cnt].GetY();
    }
    X /= (double)Count;
    Y /= (double)Count;

    Mid = EyePointD(X, Y);
}

EyePointD GlintLocation::GetMid()
{
    return Mid;
}

void GlintLocation::FindMaxRect()
{
    int MinX = 0, MinY = 0, MaxX = 0, MaxY = 0;
    MinX = Points[0].GetX();
    MinY = Points[0].GetY();
    MaxX = Points[0].GetX();
    MaxY = Points[0].GetY();

    for (int cnt = 1; cnt < Count; cnt++)
    {
        MinX = Min(MinX, Points[cnt].GetX());
        MinY = Min(MinY, Points[cnt].GetY());
        MaxX = Max(MaxX, Points[cnt].GetX());
        MaxY = Max(MaxY, Points[cnt].GetY());
    }

    int ExtraStep = 3;

    MaxRect = EyeRectangle(MinX - ExtraStep, MinY - ExtraStep, MaxX - MinX + 2* ExtraStep, MaxY - MinY + 2*ExtraStep);
}

EyeRectangle GlintLocation::GetMaxRect()
{
    return MaxRect;
}

int GlintLocation::Min(int Num1, int Num2)
{
    if (Num1 < Num2)
    {
        return Num1;
    }
    else
    {
        return Num2;
    }
}

int GlintLocation::Max(int Num1, int Num2)
{
    if (Num1 > Num2)
    {
        return Num1;
    }
    else
    {
        return Num2;
    }
}

void GlintLocation::DrawPoints(IplImage* Image)
{
    for (int cnt = 0; cnt < Count; cnt++)
    {
        EyePoint P = Points[cnt];
        int L = P.GetX() * Image->nChannels + P.GetY() * Image->width * 3;
        Image->imageData[L] = (char)0;
        Image->imageData[L + 1] = (char)-1;
        Image->imageData[L + 2] = (char)0;
    }
}
