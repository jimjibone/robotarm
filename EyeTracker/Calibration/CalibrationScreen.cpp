#include "CalibrationScreen.h"

CalibrationScreen::CalibrationScreen()
{
    Wid = 0;
    Hei = 0;
    XStep = 0;
    XStart = 0;
    YStep = 0;
    YStart = 0;
}

CalibrationScreen::~CalibrationScreen()
{

}

void CalibrationScreen::Setup(int Width, int Height)
{
    Hei = Height;
    Wid = Width;
    XStep = (int)((double)Width * 0.9 / 2.0);
    XStart = (int)((double)Width * 0.05);
    YStep = (int)((double)Height * 0.9 / 2.0);
    YStart = (int)((double)Height * 0.05);

    Points = (EyePoint*)malloc(sizeof(EyePoint) * 9);
    for (int X = 0; X < 3; X++)
    {
        for (int Y = 0; Y < 3; Y++)
        {
            Points[X + Y * 3] = EyePoint(XStart + X * XStep, YStart + YStep * Y);
        }
    }
}

void CalibrationScreen::StartCalibration(char* Name)
{
    CurPoi = 0;
    WindowName = Name;
    DrawImage();
}

bool CalibrationScreen::NextPoint()
{
    CurPoi++;
    DrawImage();
    return CurPoi >= 9;
}

int CalibrationScreen::CurPoint()
{
    return CurPoi;
}

void CalibrationScreen::DrawImage()
{
    IplImage* Image = cvCreateImage(cvSize(Wid, Hei), 8, 3);
    cvCircle(Image, cvPoint(Points[CurPoi].GetX(), Points[CurPoi].GetY()), 10, CV_RGB(255, 255, 0), -1);
    cvShowImage(WindowName, Image);
    cvReleaseImage(&Image);
}

EyePoint* CalibrationScreen::AllPoints()
{
    return Points;
}
