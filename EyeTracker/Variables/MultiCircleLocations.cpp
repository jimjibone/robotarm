#include "MultiCircleLocations.h"

MultiCircleLocations::MultiCircleLocations()
{
    Centers = (EyePoint*)malloc(sizeof(EyePoint) * MaxCircles);
    Pupils = (int*)malloc(sizeof(int) * MaxCircles);
    Irises = (int*)malloc(sizeof(int) * MaxCircles);
    CountCentre = 0;
    CountIris = 0;
    CountPupil = 0;
}

MultiCircleLocations::~MultiCircleLocations()
{
    //dtor
}

void MultiCircleLocations::AddCentre(int X, int Y)
{
    AddCentre(EyePoint(X, Y));
}

void MultiCircleLocations::AddCentre(EyePoint Loc)
{
    if (CountCentre < MaxCircles - 1)
    {
        Centers[CountCentre] = Loc;
        CountCentre++;
    }
}

void MultiCircleLocations::AddIrisRadius(int Radius)
{
    if (CountIris < MaxCircles - 1)
    {
        Irises[CountIris] = Radius;
        CountIris++;
    }
}

void MultiCircleLocations::AddPupilRadius(int Radius)
{
    if (CountPupil < MaxCircles - 1)
    {
        Pupils[CountPupil] = Radius;
        CountPupil++;
    }
}

void MultiCircleLocations::AverageCircles()
{
    int IrisTotal = 0, PupilTotal = 0, XTotal = 0, YTotal = 0;

    //printf("Iris Total: %d\nPupil Total: %d\nCenter Total: %d", CountIris, CountPupil, CountCentre);

    for (int cnt = 0; cnt < CountCentre; cnt++)
    {
        XTotal += Centers[cnt].GetX();
        YTotal += Centers[cnt].GetY();
    }

    for (int cnt = 0; cnt < CountIris; cnt++)
    {
        IrisTotal += Irises[cnt];
    }

    for (int cnt = 0; cnt < CountPupil; cnt++)
    {
        PupilTotal += Pupils[cnt];
    }

    IrisRadius = (double)IrisTotal / (double)CountIris;
    PupilRadius = (double)PupilTotal / (double)CountPupil;
    double XAv = (double)XTotal / (double)CountCentre;
    double YAv = (double)YTotal / (double)CountCentre;

    //printf("Centre (%0.3f, %0.3f)\nPupil Radius: %0.3f, Iris Radius: %0.3f", XAv, YAv, PupilRadius, IrisRadius);

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
