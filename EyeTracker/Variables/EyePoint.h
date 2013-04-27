#ifndef POINT_H
#define POINT_H

#include <iostream>

class EyePoint
{
public:
    EyePoint();
    EyePoint(int, int);
    virtual ~EyePoint();

    int GetX();
    int GetY();
protected:
private:
    int X, Y;
};

class EyePointD
{
public:
    EyePointD();
    EyePointD(double, double);
    virtual ~EyePointD();

    double GetX();
    double GetY();

    int GetXint();
    int GetYint();
protected:
private:
    double X, Y;
};

class AvEyePointD
{
public:
    AvEyePointD();
    AvEyePointD(int);
    virtual ~AvEyePointD();

    void AddPoint(EyePointD);
    EyePointD GetCurAverage();

    void Reset();

protected:
private:

    int NumAt;
    int MaxNum;
    bool First;

    EyePointD* Points;
};


#endif // POINT_H
