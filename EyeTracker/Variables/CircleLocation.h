#ifndef CIRCLELOCATION_H
#define CIRCLELOCATION_H

#include "EyeRectangle.h"
#include "EyePoint.h"

//A function to hold the data for the eye location
//Works out some future values aswell
class CircleLocation
{
public:
    CircleLocation();
    CircleLocation(int, int, int);
    CircleLocation(int, int, int, int);
    CircleLocation(EyePoint, int);
    CircleLocation(EyePoint, int, int);
    virtual ~CircleLocation();

    EyeRectangle GetInnerRectangle();
    EyeRectangle GetOutterRectangle();

    int GetX();
    int GetY();
    EyePoint CentreLoc();
    int GetInnerRadius();
    int GetOutterRadius();
    bool IrisFound();
protected:
private:
    //Variables
    EyePoint Center;
    int InR, OutR;
    EyeRectangle InRect, OutRect;

    //Functions
    void SetRect(EyePoint, int, int);
};

#endif // CIRCLELOCATION_H
