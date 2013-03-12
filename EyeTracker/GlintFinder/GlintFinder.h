#ifndef GLINTFINDER_H
#define GLINTFINDER_H

#include "../CircleFinder/Variables/Rectangle.h"
#include "Variables/EyeRange.h"
#include "Variables/GlintLocation.h"
#include "Variables/Point.h"
#include "../CircleFinder/Variables/CircleLocation.h"
#include <opencv/cv.h>

class GlintFinder
{
    public:
        GlintFinder();
        GlintFinder(int NumToExpect);
        virtual ~GlintFinder();

        void FindGlints(IplImage* Image);
        void FindGlints(IplImage* Image, CircleLocation Location);
        GlintLocation GetGlintLocation(int Num);

        void DrawGlints(IplImage* Image);
    protected:
    private:
    int NumToExptect;
    GlintLocation* GlintsLocs;
    EyeRange CurRange;

    void FindGlints(IplImage* Image, EyeRectangle Rect);

    void Around(EyePoint Loc, int From, int Num, IplImage* Image);
    EyeRectangle ConstrainRect(EyeRectangle Rect, IplImage* Image);
};

#endif // GLINTFINDER_H
