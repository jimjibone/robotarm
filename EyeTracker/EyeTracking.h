#ifndef EYETRACKING_H
#define EYETRACKING_H

#include "PSEye/PSEye_OpenCV.h"
#include "EyeTracking/Tracking.h"
#include "Calibration/Calibration.h"
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <pthread.h>

class EyeTracking
{
    public:
        EyeTracking();
        virtual ~EyeTracking();

        void Run();
        void RunBehind();
        void StopBehind();
    protected:
    private:
        PSEye_OpenCV Cam;
        Tracking Tracker;
        Calibration Cali;

        pthread_t bk_Runner;
        static void* bk_Working(void*);
        bool Running;

        static void UpdatedImage(IplImage*, void*);
        static void UpdatedLocations(void*);
};

#endif // EYETRACKING_H
