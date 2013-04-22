#ifndef HOUGHCIRCLEFNDER_H
#define HOUGHCIRCLEFNDER_H

#include <math.h>
#include <opencv/cv.h>
#include "../../Variables/CircleLocation.h"
#include "../../Variables/MultiCircleLocations.h"
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

    MultiCircleLocations GetCircleLocation();
    int GetNumFound();

    void DrawEye(IplImage*);
protected:
private:
    //Clases
    MultiCircleLocations Locs;
    RectangleSearcher Search;

    //Constants
    static const int InnerMinR = 15;
    static const int InnerMaxR = 40;
    static const int CircleStep = 15;
    static const int CheckStep = 4;

    static const int OuterMinR = 45;
    static const int OuterMaxR = 100;
    static const int SecStep = 2;
    static const int CircleAccept = 7;

    //Variabled
    bool CheckSec;
    bool* Values;
    int Wid, Hei;
    int NumFound;

    //Functions
    int FindSecond(int, int);
    void FindCircle(EyeRectangle);

    bool CheckCircle(int, int, int);
    void FindAllCircles(int, int, int);
    void FindAllCircles(int, int, int, int);
};

#endif // HOUGHCIRCLEFNDER_H
