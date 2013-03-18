#include "HoughCircleFnder.h"

HoughCircleFnder::HoughCircleFnder()
{
    Ready = false;
}

HoughCircleFnder::HoughCircleFnder(int Width, int Height, bool SecCheck)
{
    Search = RectangleSearcher(Width, Height);
    Values = (bool*)malloc(sizeof(bool) * Width * Height); //Using Bool values for speed, they take less time to calculate
    NumFound = 0;
    Wid = Width;
    Hei = Height;
    CheckSec = SecCheck;
    Ready = true;
}

HoughCircleFnder::~HoughCircleFnder()
{

}


bool HoughCircleFnder::GetSecChecker()
{
    return CheckSec;
}

void HoughCircleFnder::SetSecChecker(bool Check)
{
    CheckSec = Check;
}


int HoughCircleFnder::GetNumFound()
{
    return NumFound;
}

CircleLocation HoughCircleFnder::GetCircleLocation()
{
    return Loc;
}


void HoughCircleFnder::SetOpenCVImage(IplImage* Image)
{//Binary Image is required, with values of either 0 or -1 (stupid signed char thing)
    for (int X = 0; X < Wid; X++)
    {
        for (int Y = 0; Y < Hei; Y++)
        {
            int P = Y * Wid + X;
            Values[P] = Image->imageData[P] == -1;
        }
    }
}


void HoughCircleFnder::FindCircle()
{//Function decides if to look in a small area or scan the whole image again.
    if (NumFound != 0)
    {
        FindCircle(Loc.NextRectangle());
        return;
    }

    //First check middle of image then move out from that, covering all points on an image
    //This is to speed up, assumes its more likely an eye is in the middle (or near) of an image
    for (int i = 0; i < Search.Length; i++)
    {
        FindCircle(Search.Rects[i]);
        if (NumFound != 0) break;
    }
}

void HoughCircleFnder::FindCircle(EyeRectangle Rect)
{
    NumFound = 0;
    for (int R = InnerMinR; R <= InnerMaxR; R++)
    { //R first in hope its a bit quicker to scan
        for (int X = Rect.Left(); X < Rect.Right(); X++)
        {
            for (int Y = Rect.Top(); Y < Rect.Bottom(); Y++)
            {
                bool Found = true; //Define circle to be found, and only discount when proven
                int Test = 0;
                for (int P = 0; P < 360; P += CircleStep)
                {//Using Houghs Transformations Circle Theroem to find points on a circle
                    double Theta = (double)P * Pi / 180.0;
                    int TX = X + (int)((double)R * cos(Theta));
                    int TY = Y + (int)((double)R * sin(Theta));
                    if (TX > 0 && TX < Wid && TY > 0 && TY < Hei)
                    { // Check if point is within image size
                        if (!Values[TX + TY * Wid])
                        { // Allows 2 points on the circle to be missed for robustness
                            Test++;
                        }
                    }
                    else
                    {
                        Found = false;
                        break;
                    }

                    if (Test > 2)
                    { // If 2 points are missed, breaks and tries next point
                        Found = false;
                        break;
                    }
                }
                if (Found)
                {//When circle not discounted either define as or check if a 2nd circle representing iris if found
                    if (CheckSec)
                    {
                        int Sec = FindSecond(X, Y);
                        if (Sec != -1)
                        {
                            NumFound = 1;
                            Loc = CircleLocation(X, Y, R, Sec);
                            return;
                        }//If 2nd circle not found move on as normal and try again
                    }
                    else
                    {
                        NumFound = 1;
                        Loc = CircleLocation(X, Y, R);
                        return;
                    }
                }
            }
        }
    }
}

int HoughCircleFnder::FindSecond(int X, int Y)
{//Checking a small area around the point for robustness
    for (int x = X - SecStep; x < X + SecStep; x++)
    {
        for (int y = Y - SecStep; y < Y + SecStep; y++)
        {
            for (int R = OuterMinR; R <= OuterMaxR; R++)
            {
                int Num = 0;
                //Test is differant this time, as not expecting the whole iris to be present
                //Circle needs to have a defined numer more succeses than fails to be found
                for (int P = 0; P < 360; P += CircleStep)
                {
                    //Again using Houghs
                    double Theta = (double)P * Pi / 180.0;
                    int TX = x + (int)((double)R * cos(Theta));
                    int TY = y + (int)((double)R * sin(Theta));
                    if (Values[TX + TY * Wid])
                    {
                        Num++;
                    }

                    if (Num > CircleAccept)
                    {//When circle is found to be correct return radius of that circle
                        return R;
                    }
                }
            }
        }
    }
    return -1;
}


void HoughCircleFnder::DrawImage(IplImage* Image)
{//Image draws the bool data onto a defined image, to check it is setting cirrectly
    //Image needs to be B&W
    for (int X = 0; X < Wid ; X++)
    {
        for (int Y = 0; Y < Hei; Y++)
        {
            int p = X + Y * Wid;
            if (Values[p])
            {
                Image->imageData[p] = (char)100;
            }
            else
            {
                Image->imageData[p] = (char)0;
            }
        }
    }
}

void HoughCircleFnder::DrawEye(IplImage* Image)
{//Image needs to be RGB or ARGB
    if (GetNumFound() == 1)
    {
        if (Loc.IrisFound())
        {
            cvCircle(Image, cvPoint(Loc.GetX(), Loc.GetY()), Loc.GetOutterRadius(), CV_RGB(0,255,0), 2, 8, 0);
        }
        cvCircle(Image, cvPoint(Loc.GetX(), Loc.GetY()), Loc.GetInnerRadius(), CV_RGB(255,0,0), 2, 8, 0);
    }
}
