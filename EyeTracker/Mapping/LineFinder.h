#ifndef LINEFINDER_H
#define LINEFINDER_H

#include "Linear.h"
#include "Quadratic.h"
#include "../Variables/EyeDifferance.h"
#include "../Variables/EyePoint.h"

enum LineType {LinearLine, QuadraticLine};

class LineFinder
{
    public:
        LineFinder();
        virtual ~LineFinder();

        void UpdateLineType(LineType);

        void UpdateCalibration(EyeDifferance*, EyePoint[]);

        EyePointD FindPoint(EyeDifferance);
    protected:
    private:
};

#endif // LINEFINDER_H
