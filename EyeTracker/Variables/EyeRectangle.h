#ifndef RECTANGLE_H
#define RECTANGLE_H

class EyeRectangle
{
public:
    EyeRectangle();
    EyeRectangle(int, int);
    EyeRectangle(int, int, int, int);
    virtual ~EyeRectangle();

    int Left();
    int Right();
    int Top();
    int Bottom();
    int X();
    int Y();
    int Width();
    int Height();
protected:
private:
    int L, R, T, B, iX, iY, W, H;
};

class EyeRectangleD
{
public:
    EyeRectangleD();
    EyeRectangleD(double, double);
    EyeRectangleD(double, double, double, double);
    virtual ~EyeRectangleD();

    double Left();
    double Right();
    double Top();
    double Bottom();
    double X();
    double Y();
    double Width();
    double Height();

    int intLeft();
    int intRight();
    int intTop();
    int intBottom();
    int intX();
    int intY();
    int intWidth();
    int intHeight();
protected:
private:
    double L, R, T, B, dX, dY, W, H;
    int iL, iR, iT, iB, iX, iY, iW, iH;
};

#endif // RECTANGLE_H
