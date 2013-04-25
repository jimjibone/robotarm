#ifndef TRACKING_H
#define TRACKING_H

#include <pthread.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include "CircleFinder/HoughCircleFnder.h"
#include "GlintFinder/GlintFinder.h"
#include "../Variables/EyeDifferance.h"

typedef void (*PosUpdate)(bool, EyeDifferance, void*);

class Tracking
{
public:
    Tracking();
    virtual ~Tracking();

    void Setup(int, int, PosUpdate, void*);

    void Track(IplImage*, IplImage*);

    EyePointD GetGlintLocation();
    EyePointD GetCircleLocation();

    void ShowWindow();
    void HideWindow();

    int GetNumOfWindows();
protected:
private:
    IplImage* CurImage;
    IplImage* OrigImage;

    void* SentData;
    PosUpdate UpdateFuncs;

    pthread_t bk_Process;
    static void* bk_Process_Thread(void*);
    bool Running;

    bool ShowWind;

    HoughCircleFnder CircleFinder;
    GlintFinder GlintsFinder;
};

#endif // TRACKING_H
