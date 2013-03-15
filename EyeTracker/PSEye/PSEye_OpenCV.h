#ifndef PSEYE_OPENCV_H
#define PSEYE_OPENCV_H

#include "PSEyeGetter.h"
#include "EyeTimers.h"
#include "opencv/cv.h"
#include <opencv/highgui.h>
#include <pthread.h>

bool Something();

class PSEye_OpenCV
{
    public:
        PSEye_OpenCV();

        virtual ~PSEye_OpenCV();

        bool FindCamera();
        bool StartCapture();
        void StopCapture();

        void StartOtherCapture();
        void StopOtherCapture();

        void ShowImage();
        void HideImage();

        IplImage* GetImage();
    protected:
    private:
        PSEyeGetter Cam;
        EyeTimers ThisTimer;

        pthread_t Line_Thread;

        int CurImage;
        IplImage* Img_1;
        IplImage* Img_2;
        IplImage* Img_3;

        static void DoItFunction(void* ptr);

        static void* InLineFunnction(void* Input);

        bool StillRunning;
        bool InLineRunning;
        bool ShowWind;
        bool Ready;
};

#endif // PSEYE_OPENCV_H
