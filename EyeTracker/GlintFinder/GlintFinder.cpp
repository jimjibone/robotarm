#include "GlintFinder.h"

GlintFinder::GlintFinder()
{
    //ctor
}

GlintFinder::GlintFinder(int NumtoExpect)
{
    NumToExptect = NumtoExpect;
    GlintsLocs = (GlintLocation*)malloc(NumToExptect * sizeof(GlintLocation));
}

GlintFinder::~GlintFinder()
{
    //dtor
}

void GlintFinder::FindGlints(IplImage* Image, CircleLocation Location)
{
    FindGlints(Image, Location.BrightRectangle());
}

void GlintFinder::FindGlints(IplImage* Image, EyeRectangle Rect)
{
    for (int cnt = 0; cnt < NumToExptect; cnt++)
    {
        GlintsLocs[cnt] = GlintLocation();
    }

    for (int cnt = 0; cnt < NumToExptect; cnt++)
    {
        unsigned char Max = 0;
        EyePoint MaxLoc;
        for (int X = Rect.Left(); X < Rect.Right(); X++)
        {
            for (int Y = Rect.Top(); Y < Rect.Bottom(); Y++)
            {
                int P = X + Y * Image->width;
                bool Carry = true;
                for (int ctr = 0; ctr < cnt; ctr++)
                {
                    if (!GlintsLocs[ctr].WithinOld(EyePoint(X, Y)))
                    {
                        Carry = false;
                        break;
                    }
                }
                if (Carry)
                {
                    if (Max < (unsigned char)Image->imageData[P])
                    {
                        Max = (unsigned char)Image->imageData[P];
                        MaxLoc = EyePoint(X, Y);
                    }
                }
            }
        }

        CurRange = EyeRange(Max);
        GlintsLocs[cnt].AddPoint(MaxLoc);

        Around(MaxLoc, 1, cnt, Image);

        GlintsLocs[cnt].FindMaxRect();
        GlintsLocs[cnt].FindMid();
    }
}

GlintLocation GlintFinder::GetGlintLocation(int Num)
{
    if (Num >= NumToExptect)
    {
        return GlintLocation();
    }
    else
    {
        return GlintsLocs[Num];
    }
}

void GlintFinder::Around(EyePoint Loc, int From, int Nums, IplImage* Image)
{
    int Num = 0;
    EyeRectangle Rect = EyeRectangle(Loc.GetX() - From, Loc.GetY() - From, From * 2, From * 2);
    if (!(Rect.Left() > 0 && Rect.Right() < Image->width && Rect.Top() > 0 && Rect.Bottom() < Image->height))
    {
        //printf("Argument Got too Large\n");
        return;
    }

    for (int X = Rect.Left(); X <= Rect.Right(); X++)
    {
        int P = X + Rect.Top() * Image->width;
        if (CurRange.CheckWithin(Image->imageData[P]))
        {
            GlintsLocs[Nums].AddPoint(EyePoint(X, Rect.Top()));
            Num++;
        }

        P = X + Rect.Bottom() * Image->width;
        if (CurRange.CheckWithin(Image->imageData[P]))
        {
            GlintsLocs[Nums].AddPoint(EyePoint(X, Rect.Bottom()));
            Num++;
        }
    }

    for (int Y = Rect.Top(); Y <= Rect.Bottom(); Y++)
    {
        int P = Rect.Left() + Y * Image->width;
        if (CurRange.CheckWithin(Image->imageData[P]))
        {
            GlintsLocs[Nums].AddPoint(EyePoint(Rect.Left(), Y));
            Num++;
        }

        P = Rect.Right() + Y * Image->width;
        if (CurRange.CheckWithin(Image->imageData[P]))
        {
            GlintsLocs[Nums].AddPoint(EyePoint(Rect.Right(), Y));
            Num++;
        }
    }

    if (Num != 0)
    {
        Around(Loc, From + 1, Nums, Image);
    }
}

EyeRectangle GlintFinder::ConstrainRect(EyeRectangle Rect, IplImage* Image)
{
    int NewX, NewY, Width, Height;

    if (Rect.Left() > 0)
    {
         NewX = Rect.Left();
    }
    else
    {
         NewX = 0;
    }

    if (Rect.Top() > 0)
    {
        NewY = Rect.Top();
    }
    else
    {
        NewY = 0;
    }

    if (Rect.Right() < Image->width)
    {
        Width = Rect.Right() - Rect.Left();
    }
    else
    {
        Width = Image->width - NewX;
    }

    if (Rect.Bottom() < Image->height)
    {
        Height = Rect.Bottom() - Rect.Top();
    }
    else
    {
        Height = Image->height - NewY;
    }

    return EyeRectangle(NewX, NewY, Width, Height);
}

void GlintFinder::DrawGlints(IplImage* Image)
{
    for (int cnt = 0; cnt < NumToExptect; cnt++)
    {
        EyePointD P = GlintsLocs[cnt].GetMid();
        GlintsLocs[cnt].DrawPoints(Image);
        cvCircle(Image, cvPoint(P.GetXint(), P.GetYint()), 1, CV_RGB(0, 0, 255), 2);
    }
}
