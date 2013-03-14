#include <iostream>
#include "PSTesting.h"
#include <opencv/cv.h>
#include <opencv/highgui.h>

using namespace std;

int main()
{
    PSTesting Cam = PSTesting();

    Cam.Test2();

    Cam.OpenCam();

    cvNamedWindow("Image", CV_WINDOW_AUTOSIZE);

    IplImage* img_cam =  cvCreateImage(cvSize(640, 480), 8, 1);

    while (1)
    {
        Cam.GetImage(img_cam);
        cvShowImage("Image", img_cam);

        if (cvWaitKey(10) == 27) break;
    }

    cvReleaseImage(&img_cam);
    cvDestroyAllWindows();

    return 0;
}
