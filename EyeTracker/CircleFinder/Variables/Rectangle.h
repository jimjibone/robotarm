#ifndef RECTANGLE_H
#define RECTANGLE_H

class EyeRectangle
{
public:
    EyeRectangle();
    EyeRectangle(int Width, int Height);
    EyeRectangle(int X, int Y, int Width, int Height);
    virtual ~EyeRectangle();

    int Left();
    int Right();
    int Top();
    int Bottom();
protected:
private:
    int L, R, T, B;
};

#endif // RECTANGLE_H
