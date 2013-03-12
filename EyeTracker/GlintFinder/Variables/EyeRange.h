#ifndef EYERANGE_H
#define EYERANGE_H

#include <iostream>

class EyeRange
{
    public:
        EyeRange();
        EyeRange(unsigned char Value);
        virtual ~EyeRange();

        bool CheckWithin(unsigned char Value);
        unsigned char GetUpperBoundU();
        char GetUpperBound();
        char GetLowerBound();
        unsigned char GetLowerBoundU();
    protected:
    private:
        unsigned char Upper, Lower;

        static const int RangeArea = 20;
};

#endif // EYERANGE_H
