#ifndef POINT_H
#define POINT_H


class EyePoint
{
    public:
        EyePoint();
        EyePoint(int X, int Y);
        virtual ~EyePoint();

        int GetX();
        int GetY();
    protected:
    private:
    int X, Y;
};

#endif // POINT_H
