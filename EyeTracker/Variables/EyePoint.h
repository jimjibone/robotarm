#ifndef POINT_H
#define POINT_H


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


#endif // POINT_H
