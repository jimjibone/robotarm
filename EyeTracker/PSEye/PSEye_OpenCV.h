#ifndef PSEYE_OPENCV_H
#define PSEYE_OPENCV_H

#include "PSEyeGetter.h"
#include "EyeTimers.h"
#include "opencv/cv.h"
#include <opencv/highgui.h>

bool Something();

class PSEye_OpenCV
{
    public:
        PSEye_OpenCV();

        virtual ~PSEye_OpenCV();

        bool StartCapture();
        void StopCapture();

        void ShowImage();
        void HideImage();

        IplImage* GetImage();
    protected:
    private:
        PSEyeGetter Cam;
        EyeTimers ThisTimer;

        int CurImage;
        IplImage* Img_1;
        IplImage* Img_2;
        IplImage* Img_3;

        static void DoItFunction(void* ptr);

        bool StillRunning;
        bool ShowWind;
};

#endif // PSEYE_OPENCV_H
