#include "MultiCircleLocations.h"

MultiCircleLocations::MultiCircleLocations()
{
    Circles = (CircleLocation*)malloc(sizeof(CircleLocation) * MaxCircles);
    Count = 0;
}

MultiCircleLocations::~MultiCircleLocations()
{
    //dtor
}

void MultiCircleLocations::AddCircle(CircleLocation New)
{
    if (Count < MaxCircles - 1)
    {
        Circles[Count] = New;
        Count++;
    }
}

void MultiCircleLocations::AverageCircles()
{
    int IrisTotal = 0, PupilTotal = 0, XTotal = 0, YTotal = 0;
    for (int cnt = 0; cnt < Count; cnt++)
    {
        IrisTotal += Circles[cnt].GetOutterRadius();
        PupilTotal += Circles[cnt].GetInnerRadius();
        XTotal += Circles[cnt].GetX();
        YTotal += Circles[cnt].GetY();
    }
    IrisRadius = (double)IrisTotal / (double)Count;
    PupilRadius = (double)PupilTotal / (double)Count;
    double XAv = (double)XTotal / (double)Count;
    double YAv = (double)YTotal / (double)Count;

    Center = EyePointD(XAv, YAv);
    PupilRect = EyeRectangleD(XAv - PupilRadius, YAv - PupilRadius, PupilRadius * 2.0, PupilRadius * 2.0);
    IrisRect = EyeRectangleD(XAv - IrisRadius, YAv - IrisRadius, IrisRadius * 2.0, IrisRadius * 2.0);
    NextRect = EyeRectangle(PupilRect.intLeft() - NewArea, PupilRect.intTop() - NewArea, NewArea * 2 + (int)PupilRadius * 2, NewArea * 2 + (int)PupilRadius * 2);
    BrightRect = EyeRectangle((int)XAv - (int)PupilRadius * 2, (int)YAv - (int)PupilRadius * 2, (int)PupilRadius * 4, (int)PupilRadius * 4);
}

EyeRectangleD MultiCircleLocations::PupilRectangleD()
{
    return PupilRect;
}

EyeRectangleD MultiCircleLocations::IrisRectangleD()
{
    return IrisRect;
}

EyeRectangle MultiCircleLocations::GlintSearchRectangle()
{
    return BrightRect;
}

bool MultiCircleLocations::IrisFound()
{
    return IrisRadius > 0;
}

EyeRectangle MultiCircleLocations::NextSearchRectangle()
{
    if (IrisFound())
    {
        return EyeRectangle(IrisRect.intX(), IrisRect.intY(), IrisRect.intWidth(), IrisRect.intHeight());
    }
    else
    {
        return NextRect;
    }
}

EyePointD MultiCircleLocations::CircleCenter()
{
    return Center;
}

double MultiCircleLocations::GetIrisRadius()
{
    return IrisRadius;
}

double MultiCircleLocations::GetPupilRadious()
{
    return PupilRadius;
}
