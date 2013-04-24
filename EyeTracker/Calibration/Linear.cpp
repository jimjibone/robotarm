#include "Linear.h"

Linear::Linear()
{
    m = 0;
    c = 0;
}

Linear::~Linear()
{
    //dtor
}

void Linear::FindLine(int Xi[], int Yi[], int n)
{
    int TXY = 0, TX = 0, TY = 0, TX2 = 0;
    for (int cnt = 0; cnt < n; cnt++)
    {
        TXY += Xi[cnt] * Yi[cnt];
        TX += Xi[cnt];
        TY += Yi[cnt];
        TX2 += Xi[cnt] * Xi[cnt];
    }

    int X2 = TX * TX;
    double _X = (double)TX / n;
    double _Y = (double)TY / n;

    int Top = (n * TXY) - (TX * TY);
    int Bot = (n * TX2) - X2;
    m = (double)Top / (double)Bot;
    c = _Y - (m * _X);
}

void Linear::FindLine(double[] Xi, double[] Yi, int n)
{
    double TXY = 0, TX = 0, TY = 0, TX2 = 0;
    for (int cnt = 0; cnt < n; cnt++)
    {
        TXY += Xi[cnt] * Yi[cnt];
        TX += Xi[cnt];
        TY += Yi[cnt];
        TX2 += Xi[cnt] * Xi[cnt];
    }

    double X2 = TX * TX;
    double _X = TX / n;
    double _Y = TY / n;

    double Top = (n * TXY) - (TX * TY);
    double Bot = (n * TX2) - X2;
    m = Top / Bot;
    c = _Y - (m * _X);
}

double Linear::Getm()
{
    return m;
}

double Linear::Getc()
{
    return c;
}

double Linear::WorkOut(double X)
{
    return m * X + c;
}

double Linear::WorkOut(int)
{
    return m * (double)X + c;
}
