#ifndef GLINTFINDER_H
#define GLINTFINDER_H

#include "../../Variables/EyeRectangle.h"
#include "../../Variables/EyeRange.h"
#include "../../Variables/GlintLocation.h"
#include "../../Variables/EyePoint.h"
#include "../../Variables/MultiCircleLocations.h"
#include <opencv/cv.h>

class GlintFinder
{
public:
    GlintFinder();
    virtual ~GlintFinder();

    void FindGlints(IplImage*, EyeRectangle);
    GlintLocation GetGlintLocation();

    void DrawGlints(IplImage*);
protected:
private:
    GlintLocation* Glints;

    static const int GlintToExpect = 1;

    void Around(EyePoint, int, IplImage*, EyeRange);
};

#endif // GLINTFINDER_H
