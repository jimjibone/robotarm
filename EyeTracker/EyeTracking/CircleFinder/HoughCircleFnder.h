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

    bool GetIrisChecker();
    void SetIrisChecker(bool);

    void FindCircle();

    MultiCircleLocations GetCircleLocation();
    bool Found();

    void DrawEye(IplImage*);
protected:
private:
    //Clases
    MultiCircleLocations Locs;
    RectangleSearcher Search;

    //Constants
    static const int PupilMinR = 15;
    static const int PupilMaxR = 40;
    static const int PupilAccept = 2;

    static const int IrisMinR = 45;
    static const int IrisMaxR = 100;
    static const int IrisStep = 2;
    static const int IrisAccept = 8;

    static const int CircleStep = 30;

    static const int AllStep = 3;

    //Variabled
    bool IrisCheck;
    bool* Values;
    int Wid, Hei;
    bool found;

    //Functions
    void FindPupil(EyeRectangle);
    bool CheckPupil(int, int, int);
    int FindIris(int, int);
    bool CheckIris(int, int, int);

    void FindAllCircles(int, int, int, int);
};

#endif // HOUGHCIRCLEFNDER_H
