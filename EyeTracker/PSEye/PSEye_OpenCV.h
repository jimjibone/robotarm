#ifndef PSEYE_OPENCV_H
#define PSEYE_OPENCV_H

#include "../MEngProjectFiles/PSEye/PSEyeGetter.h"
#include "opencv/cv.h"
#include "pthread.h"

bool Something();

class PSEye_OpenCV
{
    public:
        PSEye_OpenCV();

        virtual ~PSEye_OpenCV();

        bool StartCapture();
        void StopCapture();

        void GetImage(IplImage Image);
    protected:
    private:
        PSEyeGetter Cam;
        pthread_t Getter;
        pthread_t m_thread;

        static void* GetterThread(void* Data);

        bool StillRunning;
};

#endif // PSEYE_OPENCV_H
