#include "EyeRectangle.h"

EyeRectangle::EyeRectangle()
{
    L = 0;
    R = 0;
    T = 0;
    B = 0;
    iX = 0;
    iY = 0;
    W = 0;
    H = 0;
}

EyeRectangle::EyeRectangle(int Width, int Height)
{
    L = 0;
    R = Width;
    T = 0;
    B = Height;
    iX = 0;
    iY = 0;
    W = Width;
    H = Height;
}

EyeRectangle::EyeRectangle(int X, int Y, int Width, int Height)
{
    L = X;
    R = X + Width;
    T = Y;
    B = Y + Height;
    iX = X;
    iY = Y;
    W = Width;
    H = Height;
}

EyeRectangle::~EyeRectangle()
{

}

int EyeRectangle::Left()
{
    return L;
}

int EyeRectangle::Right()
{
    return R;
}

int EyeRectangle::Top()
{
    return T;
}

int EyeRectangle::Bottom()
{
    return B;
}

int EyeRectangle::X()
{
    return iX;
}

int EyeRectangle::Y()
{
    return iY;
}

int EyeRectangle::Width()
{
    return W;
}

int EyeRectangle::Height()
{
    return H;
}


EyeRectangleD::EyeRectangleD()
{
    L = 0.0;
    R = 0.0;
    T = 0.0;
    B = 0.0;
    dX = 0.0;
    dY = 0.0;
    W = 0.0;
    H = 0.0;
    iL = 0;
    iR = 0;
    iT = 0;
    iB = 0;
    iX = 0;
    iY = 0;
    iW = 0;
    iH = 0;
}

EyeRectangleD::EyeRectangleD(double Width, double Height)
{
    L = 0.0;
    R = Width;
    T = 0.0;
    B = Height;
    dX = 0.0;
    dY = 0.0;
    W = Width;
    H = Height;

    iL = (int)L;
    iR = (int)R;
    iT = (int)T;
    iB = (int)B;
    iX = (int)dX;
    iY = (int)dY;
    iW = (int)W;
    iH = (int)H;
}

EyeRectangleD::EyeRectangleD(double X, double Y, double Width, double Height)
{
    L = X;
    R = X + Width;
    T = Y;
    B = Y + Height;
    dX = X;
    dY = Y;
    W = Width;
    H = Height;

    iL = (int)L;
    iR = (int)R;
    iT = (int)T;
    iB = (int)B;
    iX = (int)dX;
    iY = (int)dY;
    iW = (int)W;
    iH = (int)H;
}

EyeRectangleD::~EyeRectangleD()
{

}

double EyeRectangleD::Left()
{
    return L;
}

double EyeRectangleD::Right()
{
    return R;
}

double EyeRectangleD::Top()
{
    return T;
}

double EyeRectangleD::Bottom()
{
    return B;
}

double EyeRectangleD::X()
{
    return dX;
}

double EyeRectangleD::Y()
{
    return dY;
}

double EyeRectangleD::Width()
{
    return W;
}

double EyeRectangleD::Height()
{
    return H;
}

int EyeRectangleD::intLeft()
{
    return iL;
}

int EyeRectangleD::intRight()
{
    return iR;
}

int EyeRectangleD::intTop()
{
    return iT;
}

int EyeRectangleD::intBottom()
{
    return iB;
}

int EyeRectangleD::intX()
{
    return iX;
}

int EyeRectangleD::intY()
{
    return iY;
}

int EyeRectangleD::intWidth()
{
    return iW;
}

int EyeRectangleD::intHeight()
{
    return iH;
}
