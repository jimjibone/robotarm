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

class EyePointD
{
    public:
        EyePointD();
        EyePointD(double X, double Y);
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
