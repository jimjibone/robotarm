#include "HoughCircleFnder.h"

HoughCircleFnder::HoughCircleFnder()
{
    found = false;
}

HoughCircleFnder::HoughCircleFnder(int Width, int Height, bool IrisChecker)
{
    Search = RectangleSearcher(Width, Height);
    Values = (bool*)malloc(sizeof(bool) * Width * Height); //Using Bool values for speed, they take less time to calculate
    found = false;
    Wid = Width;
    Hei = Height;
    IrisCheck = IrisChecker;
}

HoughCircleFnder::~HoughCircleFnder()
{

}


bool HoughCircleFnder::GetIrisChecker()
{
    return IrisCheck;
}

void HoughCircleFnder::SetIrisChecker(bool Check)
{
    IrisCheck = Check;
}


bool HoughCircleFnder::Found()
{
    return found;
}

MultiCircleLocations HoughCircleFnder::GetCircleLocation()
{
    return Locs;
}


void HoughCircleFnder::SetOpenCVImage(IplImage* Image)
{
    //Binary Image is required, with values of either 0 or -1 (stupid signed char thing)
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
{
    //Function decides if to look in a small area or scan the whole image again.
    Locs.Clear();

    if (found)
    {
        FindPupil(Locs.NextSearchRectangle());
        return;
    }

    //First check middle of image then move out from that, covering all points on an image
    //This is to speed up, assumes its more likely an eye is in the middle (or near) of an image
    for (int i = 0; i < Search.Length; i++)
    {
        FindPupil(Search.Rects[i]);
        if (found) break;
    }
}

void HoughCircleFnder::FindPupil(EyeRectangle Rect)
{
    found = false;
    for (int R = PupilMinR; R <= PupilMaxR; R++)
    {
        for (int X = Rect.Left(); X < Rect.Right(); X++)
        {
            for (int Y = Rect.Top(); Y < Rect.Bottom(); Y++)
            {
                if (CheckPupil(X, Y, R))
                {
                    if (IrisCheck)
                    {
                        int Sec = FindIris(X, Y);
                        if (Sec != -1)
                        {
                            found = true;
                            FindAllCircles(X, Y, R, Sec);
                            return;
                        }
                    }
                    else
                    {
                        found = true;
                        FindAllCircles(X, Y, R, -1);
                        return;
                    }
                }
            }
        }
    }
}

bool HoughCircleFnder::CheckPupil(int X, int Y, int R)
{
    int Test = 0;
    for (int P = 0; P < 360; P += CircleStep)
    {
        //Using Houghs Transformations Circle Theroem to find points on a circle
        double Theta = (double)P * Pi / 180.0;
        int TX = X + (int)((double)R * cos(Theta));
        int TY = Y + (int)((double)R * sin(Theta));
        if (TX > 0 && TX < Wid && TY > 0 && TY < Hei)
        {
            // Check if point is within image size
            if (!Values[TX + TY * Wid])
            {
                // Allows 2 points on the circle to be missed for robustness
                Test++;
            }
        }
        else
        {
            return false;
        }

        if (Test > PupilAccept)
        {
            // If 2 points are missed, breaks and tries next point
            return false;
        }
    }
    return true;
}

int HoughCircleFnder::FindIris(int X, int Y)
{
    //Checking a small area around the point for robustness
    for (int x = X - IrisStep; x < X + IrisStep; x++)
    {
        for (int y = Y - IrisStep; y < Y + IrisStep; y++)
        {
            for (int R = IrisMinR; R <= IrisMaxR; R++)
            {
                if (CheckIris(X, Y, R))
                {
                    return R;
                }
            }
        }
    }
    return -1;
}

bool HoughCircleFnder::CheckIris(int X, int Y, int R)
{
    int Test = 0;
    for (int P = 0; P < 360; P += CircleStep)
    {
        //Using Houghs Transformations Circle Theroem to find points on a circle
        double Theta = (double)P * Pi / 180.0;
        int TX = X + (int)((double)R * cos(Theta));
        int TY = Y + (int)((double)R * sin(Theta));
        if (TX > 0 && TX < Wid && TY > 0 && TY < Hei)
        {
            // Check if point is within image size
            if (!Values[TX + TY * Wid])
            {
                // Allows 2 points on the circle to be missed for robustness
                Test++;
            }
        }
        else
        {
            return false;
        }

        if (Test > IrisAccept)
        {
            return false;
        }
    }
    return true;
}


void HoughCircleFnder::FindAllCircles(int X, int Y, int InR, int OutR)
{
    for (int r = InR - AllStep; r < InR + AllStep; r++)
    {
        for (int x = X - AllStep; x < X + AllStep; x++)
        {
            for (int y = Y - AllStep; y < Y + AllStep; y++)
            {
                if (CheckPupil(x, y, r))
                {
                    Locs.AddCentre(x, y);
                    Locs.AddPupilRadius(r);
                }
            }
        }
    }

    if (OutR != -1)
    {
        for (int r = OutR - AllStep; r < OutR + AllStep; r++)
        {
            for (int x = X - AllStep; x < X + AllStep; x++)
            {
                for (int y = Y - AllStep; y < Y + AllStep; y++)
                {
                    if (CheckIris(x, y, r))
                    {
                        Locs.AddCentre(x, y);
                        Locs.AddIrisRadius(r);
                    }
                }
            }
        }
    }

    Locs.AverageCircles();
}


void HoughCircleFnder::DrawImage(IplImage* Image)
{
    //Image draws the bool data onto a defined image, to check it is setting cirrectly
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
{
    //Image needs to be RGB or ARGB
    if (found)
    {
        if (Locs.IrisFound())
        {
            cvCircle(Image, cvPoint(Locs.CircleCenter().GetXint(), Locs.CircleCenter().GetYint()), (int)Locs.GetIrisRadius(), CV_RGB(0,255,0), 2, 8, 0);
        }
        cvCircle(Image, cvPoint(Locs.CircleCenter().GetXint(), Locs.CircleCenter().GetYint()), (int)Locs.GetPupilRadius(), CV_RGB(255,0,0), 2, 8, 0);
        cvCircle(Image, cvPoint(Locs.CircleCenter().GetXint(), Locs.CircleCenter().GetYint()), 2, CV_RGB(0,0,255), -1);
    }
}
