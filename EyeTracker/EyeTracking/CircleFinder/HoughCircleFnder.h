#ifndef HOUGHCIRCLEFNDER_H
#define HOUGHCIRCLEFNDER_H

#include <math.h>
#include <opencv/cv.h>
#include "../../Variables/CircleLocation.h"
#include "RectangleSearcher.h"
#define Pi 3.14159265359

class HoughCircleFnder
{
public:
    HoughCircleFnder();
    HoughCircleFnder(int, int, bool);
    virtual ~HoughCircleFnder();

    void SetOpenCVImage(IplImage*);
    void DrawImage(IplImage*);

    bool GetSecChecker();
    void SetSecChecker(bool);

    void FindCircle();

    CircleLocation GetCircleLocation();
    int GetNumFound();

    void DrawEye(IplImage*);
protected:
private:
    //Clases
    CircleLocation Loc;
    RectangleSearcher Search;

    //Constants
    static const int InnerMinR = 15;
    static const int InnerMaxR = 25;
    static const int CircleStep = 15;

    static const int OuterMinR = 30;
    static const int OuterMaxR = 60;
    static const int SecStep = 2;
    static const int CircleAccept = 7;

    //Variabled
    bool CheckSec;
    bool Ready;
    bool* Values;
    int Wid, Hei;
    int NumFound;

    //Functions
    int FindSecond(int, int);
    void FindCircle(EyeRectangle);
};

#endif // HOUGHCIRCLEFNDER_H
