#ifndef QUADRATIC_H
#define QUADRATIC_H

#include <iostream>

using namespace std;

class Quadratic
{
public:
    Quadratic();
    virtual ~Quadratic();

    void FindLine(int[],int[], int);
    void FindLine(double[],double[], int);

    double Geta();
    double Getb();
    double Geac();

    double WorkOut(int):
    double WorkOut(double):
protected:
private:

    int Pow(int, int);
    double Pow(double, int);
    double a, b, c;
};

#endif // QUADRATIC_H
