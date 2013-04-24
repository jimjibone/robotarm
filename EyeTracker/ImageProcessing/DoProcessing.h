#ifndef DOPROCESSING_H
#define DOPROCESSING_H

#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <pthread.h>
#include "ImagePlaying.h"

typedef void (*FinsihedProcessing)(IplImage*, IplImage*, void*);

class DoProcessing
{
    public:
        DoProcessing();
        virtual ~DoProcessing();

        void Setup(FinsihedProcessing, void*);

        void ProcessImage(IplImage*);

        void ShowCaliWindow();
        void HideCaliWindow();

        int GetNumWindows();
    protected:
    private:
        pthread_t bk_Process;
        static void* bk_Process_Thread(void*);
        bool Running;

        IplImage* CurImage;

        FinsihedProcessing UpdateFuncs;
        void* SentData;

        bool ShowWind;
        static void MinChange(int, void*);
        static void MaxChange(int, void*);

        ImagePlaying Img_proc;
};

#endif // DOPROCESSING_H
