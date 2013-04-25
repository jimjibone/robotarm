#ifndef EYETRACKING_H
#define EYETRACKING_H

#include "PSEye/PSEye_OpenCV.h"
#include "EyeTracking/Tracking.h"
#include "Calibration/Calibration.h"
#include "ImageProcessing/DoProcessing.h"
#include "Mapping/LineFinder.h"
#include "Variables/EyeDifferance.h"
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
    bool RunCommnad(char);
    void StopBehind();
protected:
private:
    PSEye_OpenCV Cam;
    Tracking Tracker;
    Calibration Cali;
    DoProcessing Img_Proc;
    LineFinder Calcs;

    EyeDifferance CurDiff;
    bool DiffFound;

    pthread_t bk_Runner;
    static void* bk_Working(void*);
    bool Running;

    static void UpdatedImage(IplImage*, void*);
    static void UpdateProcessed(IplImage*, IplImage*, void*);
    static void UpdatedLocations(bool, EyeDifferance, void*);

    void DisplayComands();
    bool Setup();
};

#endif // EYETRACKING_H
