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

void GlintFinder::FindGlints(IplImage* Image)
{
    FindGlints(Image, EyeRectangle(0, 0, Image->width, Image->height));
}

void GlintFinder::FindGlints(IplImage* Image, CircleLocation Location)
{
    FindGlints(Image, Location.BrightRectangle());
}

void GlintFinder::FindGlints(IplImage* Image, CircleLocation Location, IplImage* Draw)
{
    FindGlints(Image, Location.BrightRectangle(), Draw);
}

void GlintFinder::FindGlints(IplImage* Image, EyeRectangle Rect)
{

}

void GlintFinder::FindGlints(IplImage* Image, EyeRectangle Rect, IplImage* DrawOnto)
{
    for (int cnt = 0; cnt < NumToExptect; cnt++)
    {
        GlintsLocs[cnt] = GlintLocation();
    }

    for (int cnt = 0; cnt < NumToExptect; cnt++)
    {
        char Max = 0, Min = 0;
        EyePoint MaxLoc, MinLoc;
        for (int X = Rect.Left(); X < Rect.Right(); X++)
        {
            for (int Y = Rect.Top(); Y < Rect.Bottom(); Y++)
            {
                int P = X + Y * Image->width;
                bool Carry = true;
                for (int ctr = 0; ctr < cnt; ctr++)
                {
                    if (GlintsLocs[ctr].WithinOld(EyePoint(X, Y)))
                    {
                        Carry = false;
                        break;
                    }
                }
                if (Carry)
                {
                    if (Max < Image->imageData[P])
                    {
                        Max = Image->imageData[P];
                        MaxLoc = EyePoint(X, Y);
                    }

                    if (Image->imageData[P] > 100)
                    {
                        //DrawOnto->imageData[P] = Image->imageData[P];
                    }

                    if (Image->imageData[P] > -50 && Image->imageData[P] < 0)
                    {
                        DrawOnto->imageData[P] = Image->imageData[P];
                    }

                    if (Min > Image->imageData[P])
                    {
                        Min = Image->imageData[P];
                        MinLoc = EyePoint(X, Y);
                    }
                }
            }
        }

        CurRange = EyeRange(Max);
        GlintsLocs[cnt].AddPoint(MaxLoc);

        printf("Max At (%d, %d) Get %d\n", MaxLoc.GetX(), MaxLoc.GetY(), Image->imageData[MaxLoc.GetX() + MaxLoc.GetY() * Image->width]);
        printf("Max At (%d, %d) Get %d\n", MinLoc.GetX(), MinLoc.GetY(), Image->imageData[MinLoc.GetX() + MinLoc.GetY() * Image->width]);

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
        printf("Argument Got too Large");
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

    printf("NumFound = %d\n", Num);

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
        EyePoint P = GlintsLocs[cnt].GetMid();
        //GlintsLocs[cnt].DrawPoints(Image);
        cvCircle(Image, cvPoint(P.GetX(), P.GetY()), 1, CV_RGB(0, 0, 255), 2);
    }
}
