#ifndef TRACKING_H
#define TRACKING_H

#include <pthread.h>
#include <opencv/cv.h>
#include "ImageProcessing/ImagePlaying.h"
#include "CircleFinder/HoughCircleFnder.h"
#include "CircleFinder/Variables/CircleLocation.h"
#include "GlintFinder/GlintFinder.h"
#include "GlintFinder/Variables/Point.h"

class Tracking
{
    public:
        Tracking();
        virtual ~Tracking();

        void CreateTracking(int Height, int Width);

        void Track(IplImage* Image);

        GlintLocation GetCurPoint();
        CircleLocation GetCurEyePoint();

        void ShowWindow();
        void HideWindow();
    protected:
    private:
        pthread_t bk_Process;
        IplImage* CurImage;

        static void* bk_Process_Thread(void* Input);

        bool ShowWind;
        bool Running;

        ImagePlaying Img_proc;
        HoughCircleFnder CircleFinder;
        GlintFinder GlintsFinder;
};

#endif // TRACKING_H
