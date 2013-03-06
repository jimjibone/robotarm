#include "GlintLocation.h"

GlintLocation::GlintLocation()
{
    Points = (EyePoint*)malloc(1000 * sizeof(EyePoint));
    Count = 0;
}

GlintLocation::~GlintLocation()
{
    //dtor
}

void GlintLocation::AddPoint(EyePoint Loc)
{
    if (Count < 999)
    {
        Points[Count] = Loc;
        Count++;
    }
    if (Count >= 1000)
    {
        printf("Error, Too Many Points In Array");
    }
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

    Mid = EyePoint((int)X, (int)Y);
}

EyePoint GlintLocation::GetMid()
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

    MaxRect = EyeRectangle(MinX - 3, MinY - 3, MaxX - MinX + 6, MaxY - MinY + 6);
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

bool GlintLocation::WithinOld(EyePoint Loc)
{
    if (Loc.GetX() > MaxRect.Left() && Loc.GetX() < MaxRect.Right() && Loc.GetY() > MaxRect.Top() && Loc.GetY() < MaxRect.Bottom())
    {
        for (int cnt = 0; cnt < Count; cnt++)
        {
            if (Loc.GetX() == Points[cnt].GetX() && Loc.GetY() == Points[cnt].GetY())
            {
                return true;
            }
        }
    }
    return false;
}
