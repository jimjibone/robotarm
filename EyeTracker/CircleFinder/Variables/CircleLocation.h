#ifndef CIRCLELOCATION_H
#define CIRCLELOCATION_H

#include "Rectangle.h"

//A function to hold the data for the eye location
//Works out some future values aswell
class CircleLocation
{
public:
    CircleLocation();
    CircleLocation(int MidX, int MidY, int InnerR);
    CircleLocation(int MidX, int MidY, int InnerR, int OuterR);
    virtual ~CircleLocation();

    EyeRectangle GetInnerRectangle();
    EyeRectangle GetOutterRectangle();
    //The rectangle around found point, that is slightly larger. Will be used to next time to speed up calculation
    //Assumes eye has not moves very far since the last frame, so can narrow search area
    EyeRectangle NextRectangle();
    //Rectangle to search for the bright point representing referance point
    EyeRectangle BrightRectangle();
    int GetX();
    int GetY();
    int GetInnerRadius();
    int GetOutterRadius();
    void SetOutterRadius(int Radius);
    bool IrisFound();
protected:
private:
    //Variables
    int X, Y, InR, OutR;
    EyeRectangle InRect, OutRect, NewRect, BrightRect;

    //Constants
    static const int NewArea = 20; //How far from the old point should look next time

    //Functions
    void SetRect(int MidX, int MidY, int InnerR, int OuterR);
};

#endif // CIRCLELOCATION_H
