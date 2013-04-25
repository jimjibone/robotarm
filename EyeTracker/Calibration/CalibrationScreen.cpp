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
}

void CalibrationScreen::StartCalibration(char* Name)
{
    CurX = 0;
    CurY = 0;
    WindowName = Name;
    DrawImage();
    AddX();
}

bool CalibrationScreen::NextPoint()
{
    DrawImage();
    AddX();
    return CurX + CurY * 3 > 9;
}

int CalibrationScreen::CurPoint()
{
    return CurX + CurY * 3;
}

void CalibrationScreen::DrawImage()
{
    IplImage* Image = cvCreateImage(cvSize(Wid, Hei), 8, 3);
    cvCircle(Image, cvPoint(XStart + CurX * XStep, YStart + YStep * CurY), 10, CV_RGB(255, 255, 0), 13);
    cvShowImage(WindowName, Image);
    cvReleaseImage(&Image);
}

void CalibrationScreen::AddX()
{
    CurX++;
    if (CurX >= 3)
    {
        CurX = 0;
        CurY++;
    }
}

EyePoint* CalibrationScreen::AllPoints()
{
    EyePoint* Points = (EyePoint*)malloc(sizeof(EyePoint) * 9);
    for (int X = 0; X < 3; X++)
    {
        for (int Y = 0; Y < 3; Y++)
        {
            Points[X + Y * 3] = EyePoint(XStart + CurX * XStep, YStart + YStep * CurY);
        }
    }
    return Points;
}
