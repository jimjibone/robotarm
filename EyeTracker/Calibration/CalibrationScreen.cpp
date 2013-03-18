#include "CalibrationScreen.h"

CalibrationScreen::CalibrationScreen()
{
    XStep = (int)((double)Sizer.GetWidth() * 0.9 / 2.0);
    XStart = (int)((double)Sizer.GetWidth() * 0.05);
    YStep = (int)((double)Sizer.GetHeight() * 0.9 / 2.0);
    YStart = (int)((double)Sizer.GetHeight() * 0.05);
}

CalibrationScreen::~CalibrationScreen()
{
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
    IplImage* Image = cvCreateImage(cvSize(Sizer.GetWidth(), Sizer.GetHeight()), 8, 3);
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
