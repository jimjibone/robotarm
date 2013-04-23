#ifndef TRACKING_H
#define TRACKING_H

#include <pthread.h>
#include <opencv/cv.h>
#include "../ImageProcessing/ImagePlaying.h"
#include "CircleFinder/HoughCircleFnder.h"
#include "GlintFinder/GlintFinder.h"
#include "../Common/EyeTimers.h"

typedef void (*PosUpdate)(void*);

class Tracking
{
    public:
        Tracking();
        virtual ~Tracking();

        void CreateTracking(int, int, PosUpdate, void*);

        void Track(IplImage*);

        GlintLocation GetGlintLocation();
        MultiCircleLocations GetCircleLocation();

        void ShowWindow();
        void HideWindow();

        void ShowSlidersWindow();
        void HideSlidersWindow();

        int GetNumOfWindows();
    protected:
    private:
        pthread_t bk_Process;
        IplImage* CurImage;

        EyeTimers Timers;
        void* SentData;

        static void* bk_Process_Thread(void*);

        PosUpdate UpdateFuncs;

        bool ShowWind;
        bool ShowTrackWind;
        bool Running;

        static void MinChange(int, void*);
        static void MaxChange(int, void*);

        ImagePlaying Img_proc;
        HoughCircleFnder CircleFinder;
        GlintFinder GlintsFinder;
};

#endif // TRACKING_H
