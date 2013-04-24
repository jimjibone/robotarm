#ifndef LINEAR_H
#define LINEAR_H

#include <iostream>
#include "../Variables/EyeDifferance.h"

using namespace std;

class Linear
{
public:
    Linear();
    virtual ~Linear();
    void FindLine(int[], int[], int);
    void FindLine(double[], double[], int);

    double Getm();
    double Getc();

    double WorkOut(double);
    double WorkOut(int);
protected:
private:
    double m, c;
};

#endif // LINEAR_H
