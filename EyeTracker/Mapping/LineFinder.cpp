#include "LineFinder.h"

LineFinder::LineFinder()
{
    ShowWind = false;
}

LineFinder::~LineFinder()
{
    //dtor
}

void LineFinder::UpdateCalibration(EyeDifferance* Differances, EyePoint* Points)
{
    PointXYZ P1X = PointXYZ(Points[0].GetX(), Points[0].GetY(), Differances[0].GetXDiff());
    PointXYZ P2X = PointXYZ(Points[1].GetX(), Points[1].GetY(), Differances[1].GetXDiff());
    PointXYZ P3X = PointXYZ(Points[2].GetX(), Points[2].GetY(), Differances[2].GetXDiff());
    PointXYZ P4X = PointXYZ(Points[3].GetX(), Points[3].GetY(), Differances[3].GetXDiff());
    PointXYZ P5X = PointXYZ(Points[4].GetX(), Points[4].GetY(), Differances[4].GetXDiff());
    PointXYZ P6X = PointXYZ(Points[5].GetX(), Points[5].GetY(), Differances[5].GetXDiff());
    PointXYZ P7X = PointXYZ(Points[6].GetX(), Points[6].GetY(), Differances[6].GetXDiff());
    PointXYZ P8X = PointXYZ(Points[7].GetX(), Points[7].GetY(), Differances[7].GetXDiff());
    PointXYZ P9X = PointXYZ(Points[8].GetX(), Points[8].GetY(), Differances[8].GetXDiff());

    PointXYZ P1Y = PointXYZ(Points[0].GetX(), Points[0].GetY(), Differances[0].GetYDiff());
    PointXYZ P2Y = PointXYZ(Points[1].GetX(), Points[1].GetY(), Differances[1].GetYDiff());
    PointXYZ P3Y = PointXYZ(Points[2].GetX(), Points[2].GetY(), Differances[2].GetYDiff());
    PointXYZ P4Y = PointXYZ(Points[3].GetX(), Points[3].GetY(), Differances[3].GetYDiff());
    PointXYZ P5Y = PointXYZ(Points[4].GetX(), Points[4].GetY(), Differances[4].GetYDiff());
    PointXYZ P6Y = PointXYZ(Points[5].GetX(), Points[5].GetY(), Differances[5].GetYDiff());
    PointXYZ P7Y = PointXYZ(Points[6].GetX(), Points[6].GetY(), Differances[6].GetYDiff());
    PointXYZ P8Y = PointXYZ(Points[7].GetX(), Points[7].GetY(), Differances[7].GetYDiff());
    PointXYZ P9Y = PointXYZ(Points[8].GetX(), Points[8].GetY(), Differances[8].GetYDiff());

    PlaneCoefficients CoefX1 = getPlaneCoefficients(P1X, P3X, P7X);
    PlaneCoefficients CoefX2 = getPlaneCoefficients(P3X, P1X, P9X);
    PlaneCoefficients CoefX3 = getPlaneCoefficients(P9X, P3X, P7X);
    PlaneCoefficients CoefX4 = getPlaneCoefficients(P7X, P9X, P1X);

    PlaneCoefficients CoefY1 = getPlaneCoefficients(P1Y, P3Y, P7Y);
    PlaneCoefficients CoefY2 = getPlaneCoefficients(P3Y, P1Y, P9Y);
    PlaneCoefficients CoefY3 = getPlaneCoefficients(P9Y, P3Y, P7Y);
    PlaneCoefficients CoefY4 = getPlaneCoefficients(P7Y, P9Y, P1Y);

    if (CoefX1.isSet())
    {
        printf("%0.3fx + %0.3fy + %0.3fz + %0.3f = 0", CoefX1.a, CoefX1.b, CoefX1.c, CoefX1.d);
    }

    if (CoefX2.isSet())
    {
        printf("%0.3fx + %0.3fy + %0.3fz + %0.3f = 0", CoefX2.a, CoefX2.b, CoefX2.c, CoefX2.d);
    }

    if (CoefX3.isSet())
    {
        printf("%0.3fx + %0.3fy + %0.3fz + %0.3f = 0", CoefX3.a, CoefX3.b, CoefX3.c, CoefX3.d);
    }

    if (CoefX4.isSet())
    {
        printf("%0.3fx + %0.3fy + %0.3fz + %0.3f = 0", CoefX4.a, CoefX4.b, CoefX4.c, CoefX4.d);
    }

    if (CoefY1.isSet())
    {
        printf("%0.3fx + %0.3fy + %0.3fz + %0.3f = 0", CoefY1.a, CoefY1.b, CoefY1.c, CoefY1.d);
    }

    if (CoefY2.isSet())
    {
        printf("%0.3fx + %0.3fy + %0.3fz + %0.3f = 0", CoefY2.a, CoefY2.b, CoefY2.c, CoefY2.d);
    }

    if (CoefY3.isSet())
    {
        printf("%0.3fx + %0.3fy + %0.3fz + %0.3f = 0", CoefY3.a, CoefY3.b, CoefY3.c, CoefY3.d);
    }

    if (CoefY4.isSet())
    {
        printf("%0.3fx + %0.3fy + %0.3fz + %0.3f = 0", CoefY4.a, CoefY4.b, CoefY4.c, CoefY4.d);
    }
}

EyePointD LineFinder::FindPoint(EyeDifferance Diff)
{

    return EyePointD();
}

void LineFinder::ShowWindow()
{
    if (!ShowWind)
    {
        cvNamedWindow("Current Point", CV_WINDOW_FULLSCREEN);
        cvMoveWindow("Current Point", 0, 0);
        Sizer.SetWindowToFullScreen("Current Point");
        ShowWind = true;
    }
}

void LineFinder::HideWindow()
{
    if (ShowWind)
    {
        ShowWind = false;
        cvDestroyWindow("Current Point");
    }
}

void LineFinder::UpdateWindow(EyePointD Point)
{
    if (ShowWind)
    {
        IplImage* Image = cvCreateImage(cvSize(Sizer.GetWidth(), Sizer.GetHeight()), 8, 3);
        cvCircle(Image, cvPoint(Point.GetXint(), Point.GetYint()), 15, CV_RGB(255, 255, 0), 0);
        cvShowImage("Current Point", Image);
        cvReleaseImage(&Image);
    }
}

int LineFinder::GetNumWindows()
{
    if (ShowWind) return 1;
    return 0;
}
