#ifndef TRACKING_H
#define TRACKING_H

#include <pthread.h>
#include <opencv/cv.h>
#include "ImageProcessing/ImagePlaying.h"
#include "CircleFinder/HoughCircleFnder.h"
#include "CircleFinder/Variables/CircleLocation.h"
#include "GlintFinder/GlintFinder.h"
#include "GlintFinder/Variables/Point.h"
#include "PSEye/PSEye_OpenCV.h"

class Tracking
{
    public:
        Tracking();
        Tracking(PSEye_OpenCV* Input);
        virtual ~Tracking();

        void StartTracking();
        void StopTracking();

        GlintLocation GetCurPoint();
        CircleLocation GetCurEyePoint();
    protected:
    private:
        pthread_t bk_Process;
        PSEye_OpenCV* InputsFrom;

        static void* bk_Process_Thread(void* Input);

        bool Runing;

        ImagePlaying Img_proc;
        HoughCircleFnder CircleFinder;
        GlintFinder GlintsFinder;
};

#endif // TRACKING_H
