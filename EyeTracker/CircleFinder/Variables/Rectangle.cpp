#include "Rectangle.h"

EyeRectangle::EyeRectangle()
{
    L = 0;
    R = 0;
    T = 0;
    B = 0;
}

EyeRectangle::EyeRectangle(int Width, int Height)
{
    L = 0;
    R = Width;
    T = 0;
    B = Height;
}

EyeRectangle::EyeRectangle(int X, int Y, int Width, int Height)
{
    L = X;
    R = X + Width;
    T = Y;
    B = Y + Height;
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
