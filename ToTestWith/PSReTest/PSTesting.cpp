#include "PSTesting.h"

PSTesting::PSTesting()
{

}

PSTesting::~PSTesting()
{
    //dtor
}

void PSTesting::OpenCam()
{
    Cam = PSEyeGetter(CLEYE_MONO_PROCESSED, CLEYE_VGA, 30);
}

void PSTesting::GetImage(IplImage* Image)
{
    if (Cam.GetFrame())
    {
        int Length = Cam.W * Cam.H * Cam.Size;
        for (int cnt = 0; cnt < Length; cnt++)
        {
            Image->imageData[cnt] = Cam.CurrentColours[cnt];
        }
    }
}

void PSTesting::Test2()
{
    cvNamedWindow("Image", CV_WINDOW_AUTOSIZE);
    PSEyeGetter Get = PSEyeGetter(CLEYE_MONO_PROCESSED, CLEYE_VGA, 30);

    IplImage* img_cam =  cvCreateImage(cvSize(Get.W, Get.H), 8, Get.Size);

    while (1)
    {
        if (Get.GetFrame())
        {
            int Length = Get.W * Get.H * Get.Size;
            for (int cnt = 0; cnt < Length; cnt++)
            {
                img_cam->imageData[cnt] = Get.CurrentColours[cnt];
            }
        }
        cvShowImage("Image", img_cam);

        if (cvWaitKey(10) == 27) break;
    }

    cvReleaseImage(&img_cam);
    cvDestroyAllWindows();
}
