#ifndef PSTESTING_H
#define PSTESTING_H

#include "../MEngProjectFiles/PSEye/PSEyeGetter.h"
#include <opencv/cv.h>
#include <opencv/highgui.h>

class PSTesting
{
    public:
        PSTesting();
        virtual ~PSTesting();

        void OpenCam();
        void GetImage(IplImage* Image);

        void Test2();
    protected:
    private:

    PSEyeGetter Cam;
};

#endif // PSTESTING_H
