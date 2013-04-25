#ifndef EYERANGE_H
#define EYERANGE_H

#include <iostream>

class EyeRange
{
public:
    EyeRange();
    EyeRange(unsigned char);
    virtual ~EyeRange();

    bool CheckWithin(unsigned char);
    unsigned char GetUpperBoundU();
    char GetUpperBound();
    char GetLowerBound();
    unsigned char GetLowerBoundU();
protected:
private:
    unsigned char Upper, Lower;
    static const int RangeArea = 30;
};

#endif // EYERANGE_H
