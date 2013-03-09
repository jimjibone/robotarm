#ifndef GLINTFINDER_H
#define GLINTFINDER_H

#include "../CircleFinder/Variables/Rectangle.h"
#include "Variables/EyeRange.h"
#include "Variables/GlintLocation.h"
#include "Variables/Point.h"
#include <opencv/cv.h>

class GlintFinder
{
    public:
        GlintFinder();
        GlintFinder(int NumToExpect);
        virtual ~GlintFinder();

        void FindGlints(IplImage* Image);
        GlintLocation GetGlintLocation(int Num);
    protected:
    private:
    int NumToExptect;
    GlintLocation* GlintsLocs;
    EyeRange CurRange;

    void Around(EyePoint Loc, int From, int Num, IplImage* Image);
    EyeRectangle ConstrainRect(EyeRectangle Rect, IplImage* Image);
};

#endif // GLINTFINDER_H
