#ifndef GLINTFINDER_H
#define GLINTFINDER_H

#include "../../Variables/EyeRectangle.h"
#include "../../Variables/EyeRange.h"
#include "../../Variables/GlintLocation.h"
#include "../../Variables/EyePoint.h"
#include "../../Variables/CircleLocation.h"
#include <opencv/cv.h>

class GlintFinder
{
    public:
        GlintFinder();
        GlintFinder(int);
        virtual ~GlintFinder();

        void FindGlints(IplImage*, CircleLocation);
        GlintLocation GetGlintLocation(int);

        void DrawGlints(IplImage*);
    protected:
    private:
    int NumToExptect;
    GlintLocation* GlintsLocs;
    EyeRange CurRange;

    void FindGlints(IplImage*, EyeRectangle);

    void Around(EyePoint, int, int, IplImage*);
    EyeRectangle ConstrainRect(EyeRectangle, IplImage*);
};

#endif // GLINTFINDER_H
