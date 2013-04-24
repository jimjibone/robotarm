#ifndef LINEFINDER_H
#define LINEFINDER_H

#include "Linear.h"
#include "Quadratic.h"
#include "../Variables/EyeDifferance.h"
#include "../Variables/EyePoint.h"

enum LineType {LinearLine, QuadraticLine};

class LineFInder
{
    public:
        LineFInder();
        virtual ~LineFInder();

        void UpdateLineType(LineType);

        void UpdateCalibration(EyeDifferance*, EyePoint[]);

        EyePointD FindPoint(EyeDifferance);
    protected:
    private:

    Linear
};

#endif // LINEFINDER_H
