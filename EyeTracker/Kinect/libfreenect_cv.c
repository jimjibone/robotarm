#include "libfreenect.h"
#include "libfreenect_sync.h"
#include "libfreenect_cv.h"

IplImage *freenect_sync_get_depth_cv(int index)
{
    IplImage* image = cvCreateImage(cvSize(640, 480), 16, 1);
    //if (!image) image = cvCreateImageHeader(cvSize(640,480), 16, 1);
    char *data = 0;
    uint32_t timestamp = 0;
    if (freenect_sync_get_depth((void**)&data, &timestamp, index, FREENECT_DEPTH_11BIT))
        return NULL;
    cvSetData(image, data, 640*2);
    return image;
}

IplImage *freenect_sync_get_rgb_cv(int index)
{
    IplImage *image = cvCreateImage(cvSize(640, 480), 8, 3);
    //if (!image) image = cvCreateImageHeader(cvSize(640,480), 8, 3);
    char *data = 0;
    uint32_t timestamp = 0;
    if (freenect_sync_get_video((void**)&data, &timestamp, index, FREENECT_VIDEO_RGB))
        return NULL;
    cvSetData(image, data, 640*3);
    return image;
}

IplImage *freenect_sync_get_ir_cv(int index, IplImage* image)
{
    //IplImage *image = cvCreateImage(cvSize(640, 480), 8, 1);
    //if (!image) image = cvCreateImageHeader(cvSize(640,480), 8, 1);
    char *data = 0;
    uint32_t timestamp = 0;
    if (freenect_sync_get_video((void**)&data, &timestamp, index, FREENECT_VIDEO_IR_8BIT))
        return NULL;
    cvSetData(image, data, 640);
    return image;
}
