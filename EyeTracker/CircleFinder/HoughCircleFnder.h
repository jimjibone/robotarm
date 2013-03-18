#ifndef HOUGHCIRCLEFNDER_H
#define HOUGHCIRCLEFNDER_H

#include <math.h>
#include <opencv/cv.h>
#include "Variables/CircleLocation.h"
#include "RectangleSearcher.h"
#define Pi 3.14159265359

class HoughCircleFnder
{
public:
    HoughCircleFnder();
    HoughCircleFnder(int Width, int Height, bool SecCheck);
    virtual ~HoughCircleFnder();

    void SetOpenCVImage(IplImage* Image);
    void DrawImage(IplImage* Image);

    bool GetSecChecker();
    void SetSecChecker(bool Check);

    void FindCircle();

    CircleLocation GetCircleLocation();
    int GetNumFound();

    void DrawEye(IplImage* Image);
protected:
private:
    int NumFound;
    CircleLocation Loc;

    RectangleSearcher Search;

    bool* Values;
    int Wid, Hei;

    //Constants
    static const int InnerMinR = 9;
    static const int InnerMaxR = 25;
    static const int CircleStep = 15;

    static const int OuterMinR = 30;
    static const int OuterMaxR = 60;
    static const int SecStep = 2;
    static const int CircleAccept = 10;

    bool CheckSec;
    bool Ready;

    //Functions
    int FindSecond(int X, int Y);
    void FindCircle(EyeRectangle Rect);
};

#endif // HOUGHCIRCLEFNDER_H
