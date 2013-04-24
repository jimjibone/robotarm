#include "Quadratic.h"

Quadratic::Quadratic()
{
    a = 0;
    b = 0;
    c = 0;
}

Quadratic::~Quadratic()
{
    //dtor
}

void Quadratic::FindLine(int Xi[], int Yi[], int n)
{
    int TX4 = 0, TX3 = 0, TX2 = 0, TX = 0, TX2Y = 0, TXY = 0, TY = 0;
    for (int cnt = 0; cnt < n; cnt++)
    {
        TX4 += Pow(Xi[cnt], 4);
        TX3 += Pow(Xi[cnt], 3);
        TX2 += Pow(Xi[cnt], 2);
        TX += Xi[cnt];
        TX2Y += Pow(Xi[cnt], 2) * Yi[cnt];
        TXY += Xi[cnt] * Yi[cnt];
        TY += Yi[cnt];
    }

    int D1 = TX4 * (TX2 * n - Pow(TX, 2)),
             D2 = TX3 * (TX3 * n - TX * TX2),
                  D3 = TX2 * (TX3 * TX - Pow(TX2, 2));
    int D = D1 - D2 + D3;
    int Da1 = TX2Y * (TX2 * n - Pow(TX, 2)),
              Da2 = TXY * (TX3 * n - TX * TX2),
                    Da3 = TY * (TX3 * TX - Pow(TX2, 2));
    int Da = Da1 - Da2 + Da3;
    int Db1 = TX4 * (TXY * n - TY * TX),
              Db2 = TX3 * (TX2Y * n - TY * TX2),
                    Db3 = TX2 * (TX2Y * TX - TXY * TX2);
    int Db = Db1 - Db2 + Db3;
    int Dc1 = TX4 * (TX2 * TY - TX * TXY),
              Dc2 = TX3 * (TX3 * TY - TX * TX2Y),
                    Dc3 = TX2 * (TX3 * TXY - TX2 * TX2Y);
    int Dc = Dc1 - Dc2 + Dc3;


    double a = (double)Da / (double)D;
    double b = (double)Db / (double)D;
    double c = (double)Dc / (double)D;
}

int Quadratic::Pow(int Num, int Power)
{
    int Tot = Num;
    for (int cnt = 2; cnt <= Power; cnt++)
    {
        Tot *= Num;
    }
    return Tot;
}

void Quadratic::FindLine(double[] Xi,double[] Yi, int n)
{
    double TX4 = 0, TX3 = 0, TX2 = 0, TX = 0, TX2Y = 0, TXY = 0, TY = 0;
    for (int cnt = 0; cnt < n; cnt++)
    {
        TX4 += Pow(Xi[cnt], 4);
        TX3 += Pow(Xi[cnt], 3);
        TX2 += Pow(Xi[cnt], 2);
        TX += Xi[cnt];
        TX2Y += Pow(Xi[cnt], 2) * Yi[cnt];
        TXY += Xi[cnt] * Yi[cnt];
        TY += Yi[cnt];
    }

    double D1 = TX4 * (TX2 * n - Pow(TX, 2)),
                D2 = TX3 * (TX3 * n - TX * TX2),
                     D3 = TX2 * (TX3 * TX - Pow(TX2, 2));
    double D = D1 - D2 + D3;
    double Da1 = TX2Y * (TX2 * n - Pow(TX, 2)),
                 Da2 = TXY * (TX3 * n - TX * TX2),
                       Da3 = TY * (TX3 * TX - Pow(TX2, 2));
    double Da = Da1 - Da2 + Da3;
    double Db1 = TX4 * (TXY * n - TY * TX),
                 Db2 = TX3 * (TX2Y * n - TY * TX2),
                       Db3 = TX2 * (TX2Y * TX - TXY * TX2);
    double Db = Db1 - Db2 + Db3;
    double Dc1 = TX4 * (TX2 * TY - TX * TXY),
                 Dc2 = TX3 * (TX3 * TY - TX * TX2Y),
                       Dc3 = TX2 * (TX3 * TXY - TX2 * TX2Y);
    double Dc = Dc1 - Dc2 + Dc3;


    double a = Da / D;
    double b = Db / D;
    double c = Dc / D;
}

double Quadratic::Pow(double Num, int Power)
{
    double Tot = Num;
    for (int cnt = 2; cnt <= Power; cnt++)
    {
        Tot *= Num;
    }
    return Tot;
}

double Quadratic::Geta()
{
    return a;
}

double Quadratic::Getb()
{
    return b;
}

double Quadratic::Geac()
{
    return c;
}

double Quadratic::WorkOut(int X)
{
    return a * (double)Pow(X, 2) + b * (double)X + c;
}

double Quadratic::WorkOut(double X)
{
    return a * Pow(X, 2) + b * X + c;
}
