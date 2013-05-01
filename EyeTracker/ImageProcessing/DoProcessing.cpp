#include "DoProcessing.h"

DoProcessing::DoProcessing()
{
    ShowWind = false;
    Running = false;
    Img_proc = ImagePlaying(120, 150);
    //Img_proc = ImagePlaying(100, 110);
}

DoProcessing::~DoProcessing()
{
    HideCaliWindow();
}

void DoProcessing::Setup(FinsihedProcessing Func, void* Data)
{
    SentData = Data;
    UpdateFuncs = Func;
}

void DoProcessing::ProcessImage(IplImage* Image)
{
    if (!Running)
    {
        Running = true;
        CurImage = Image;
        pthread_create(&bk_Process, NULL, &bk_Process_Thread, (void*) this);
    }
    else
    {
        cvReleaseImage(&Image);
    }
}

void* DoProcessing::bk_Process_Thread(void* Data)
{
    DoProcessing* This = (DoProcessing*)Data;
    IplImage* Image = cvCreateImage(cvGetSize(This->CurImage), 8, 1);

    This->Img_proc.DoAllProcesses(This->CurImage, Image);

    if (This->ShowWind) cvShowImage("Settings", Image);

    This->UpdateFuncs(This->CurImage, Image, This->SentData);

    This->Running = false;

    return NULL;
}

void DoProcessing::ShowCaliWindow()
{
    if (!ShowWind)
    {
        ShowWind = true;
        cvNamedWindow("Settings", CV_WINDOW_AUTOSIZE);
        int Min = Img_proc.GetNum1();
        int Max = Img_proc.GetNum2();
        cvCreateTrackbar2("Min", "Settings", &Min, 255, &MinChange, (void*) this);
        cvCreateTrackbar2("Max", "Settings", &Max, 255, &MaxChange, (void*) this);
    }
}

void DoProcessing::HideCaliWindow()
{
    if (ShowWind)
    {
        ShowWind = false;
        cvDestroyWindow("Settings");
    }
}

void DoProcessing::MinChange(int Num, void* ptr)
{
    ((DoProcessing*)ptr)->Img_proc.SetNum1(Num);
}

void DoProcessing::MaxChange(int Num, void* ptr)
{
    ((DoProcessing*)ptr)->Img_proc.SetNum2(Num);
}

int DoProcessing::GetNumWindows()
{
    int Num = 0;

    if (ShowWind) Num++;

    return Num;
}
