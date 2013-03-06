#ifndef EYERANGE_H
#define EYERANGE_H

#include <iostream>

class EyeRange
{
    public:
        EyeRange();
        EyeRange(char Value);
        virtual ~EyeRange();

        bool CheckWithin(char Value);
        char GetUpperBound();
        char GetLowerBound();
    protected:
    private:
        char Upper, Lower;

        static const int RangeArea = 10;
};

#endif // EYERANGE_H
