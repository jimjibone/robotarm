#include "GlintFinder.h"

GlintFinder::GlintFinder()
{
    //ctor
}

GlintFinder::~GlintFinder()
{
    //dtor
}

void GlintFinder::FindGlints(IplImage* Image, EyeRectangle Rect)
{
    GlintsLoc.Clear();

    unsigned char Max = 0;
    EyePoint MaxLoc;
    for (int X = Rect.Left(); X < Rect.Right(); X++)
    {
        for (int Y = Rect.Top(); Y < Rect.Bottom(); Y++)
        {
            int P = X + Y * Image->width;
            if (Max < (unsigned char)Image->imageData[P])
            {
                Max = (unsigned char)Image->imageData[P];
                MaxLoc = EyePoint(X, Y);
            }
        }
    }

    GlintsLoc.AddPoint(MaxLoc);

    Around(MaxLoc, 1, Image, EyeRange(Max));

    GlintsLoc.FindMaxRect();
    GlintsLoc.FindMid();
}

GlintLocation GlintFinder::GetGlintLocation()
{
    return GlintsLoc;
}

void GlintFinder::Around(EyePoint Loc, int From, IplImage* Image, EyeRange Range)
{
    int Num = 0;
    EyeRectangle Rect = EyeRectangle(Loc.GetX() - From, Loc.GetY() - From, From * 2, From * 2);
    if (!(Rect.Left() > 0 && Rect.Right() < Image->width && Rect.Top() > 0 && Rect.Bottom() < Image->height))
    {
        return;
    }

    for (int X = Rect.Left(); X <= Rect.Right(); X++)
    {
        int P = X + Rect.Top() * Image->width;
        if (Range.CheckWithin(Image->imageData[P]))
        {
            GlintsLoc.AddPoint(EyePoint(X, Rect.Top()));
            Num++;
        }

        P = X + Rect.Bottom() * Image->width;
        if (Range.CheckWithin(Image->imageData[P]))
        {
            GlintsLoc.AddPoint(EyePoint(X, Rect.Bottom()));
            Num++;
        }
    }

    for (int Y = Rect.Top(); Y <= Rect.Bottom(); Y++)
    {
        int P = Rect.Left() + Y * Image->width;
        if (Range.CheckWithin(Image->imageData[P]))
        {
            GlintsLoc.AddPoint(EyePoint(Rect.Left(), Y));
            Num++;
        }

        P = Rect.Right() + Y * Image->width;
        if (Range.CheckWithin(Image->imageData[P]))
        {
            GlintsLoc.AddPoint(EyePoint(Rect.Right(), Y));
            Num++;
        }
    }

    if (Num != 0)
    {
        Around(Loc, From + 1, Image, Range);
    }
}

void GlintFinder::DrawGlints(IplImage* Image)
{
    EyePointD P = GlintsLoc.GetMid();
    GlintsLoc.DrawPoints(Image);
    cvCircle(Image, cvPoint(P.GetXint(), P.GetYint()), 1, CV_RGB(0, 0, 255), 2);
}
