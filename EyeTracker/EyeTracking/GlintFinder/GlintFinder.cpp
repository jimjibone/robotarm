#include "GlintFinder.h"

GlintFinder::GlintFinder()
{
    Glints = (GlintLocation*)malloc(sizeof(GlintLocation) * GlintToExpect);
    for (int cnt = 0; cnt < GlintToExpect; cnt++)
    {
        Glints[cnt] = GlintLocation();
    }
}

GlintFinder::~GlintFinder()
{
    //dtor
}

void GlintFinder::FindGlints(IplImage* Image, EyeRectangle Rect)
{
    for (int cnt = 0; cnt < GlintToExpect; cnt++)
    {
        Glints[cnt].Clear();
    }

    for (int cnt = 0; cnt < GlintToExpect; cnt++)
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
                    if (Glints[ctr].CheckWithin(EyePoint(X, Y)))
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
        Glints[cnt].AddPoint(MaxLoc);

        Around(MaxLoc, cnt, Image, EyeRange(Max));

        Glints[cnt].FindMaxRect();
        Glints[cnt].FindMid();
    }
}

GlintLocation GlintFinder::GetGlintLocation()
{
    return Glints[0];
}

void GlintFinder::Around(EyePoint Loc, int Nums, IplImage* Image, EyeRange Range)
{
    for (int From = 1; From < Image->width; From++)
    {
        int Num = 0;
        EyeRectangle Rect = EyeRectangle(Loc.GetX() - From, Loc.GetY() - From, From * 2, From * 2);
        if (!(Rect.Left() > 0 && Rect.Right() < Image->width && Rect.Top() > 0 && Rect.Bottom() < Image->height))
        {
            break;
        }

        for (int X = Rect.Left(); X <= Rect.Right(); X++)
        {
            int P = X + Rect.Top() * Image->width;
            if (Range.CheckWithin(Image->imageData[P]))
            {
                Glints[Nums].AddPoint(EyePoint(X, Rect.Top()));
                Num++;
            }

            P = X + Rect.Bottom() * Image->width;
            if (Range.CheckWithin(Image->imageData[P]))
            {
                Glints[Nums].AddPoint(EyePoint(X, Rect.Bottom()));
                Num++;
            }
        }

        for (int Y = Rect.Top(); Y <= Rect.Bottom(); Y++)
        {
            int P = Rect.Left() + Y * Image->width;
            if (Range.CheckWithin(Image->imageData[P]))
            {
                Glints[Nums].AddPoint(EyePoint(Rect.Left(), Y));
                Num++;
            }

            P = Rect.Right() + Y * Image->width;
            if (Range.CheckWithin(Image->imageData[P]))
            {
                Glints[Nums].AddPoint(EyePoint(Rect.Right(), Y));
                Num++;
            }
        }

        if (Num == 0) break;
    }
}

void GlintFinder::DrawGlints(IplImage* Image)
{
    EyePointD P = Glints[0].GetMid();
    Glints[0].DrawPoints(Image);
    cvCircle(Image, cvPoint(P.GetXint(), P.GetYint()), 2, CV_RGB(0, 0, 255), -1);
}
