#ifndef PSEYE_OPENCV_H
#define PSEYE_OPENCV_H

#include "PSEyeGetter.h"
#include "EyeTimers.h"
#include "opencv/cv.h"
#include <opencv/highgui.h>
#include <pthread.h>

typedef void (*NewImage)(IplImage* Image);

class PSEye_OpenCV
{
    public:
        PSEye_OpenCV();

        virtual ~PSEye_OpenCV();

        bool FindCamera();
        bool StartCapture(NewImage NewImgFunc);
        void StopCapture();

        void ShowImage();
        void HideImage();

        int GetCameraWidth();
        int GetCameraHeight();

        IplImage* GetCurrentImage();
    protected:
    private:
        PSEyeGetter Cam;
        EyeTimers ThisTimer;

        NewImage NewImgFunct;

        static void DoItFunction(void* ptr);

        IplImage* GetImage();

        bool StillRunning;
        bool ShowWind;
};

#endif // PSEYE_OPENCV_H
