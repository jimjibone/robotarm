#include "EyeTracking.h"

EyeTracking::EyeTracking()
{
    Running = false;
}

EyeTracking::~EyeTracking()
{
    StopBehind();
}

bool EyeTracking::Setup()
{
    if (Cam.FindCamera())
    {
        Tracker.Setup(Cam.GetCameraWidth(), Cam.GetCameraHeight(), &UpdatedLocations, (void*) this);
        Img_Proc.Setup(&UpdateProcessed, (void*)this);
        Cam.StartCapture(&UpdatedImage, (void*)this);

        DisplayComands();
        return true;
    }
    else
    {
        printf("Could Not Locate Camera\nProgram Wil Exit\n");
        return false;
    }
}

void EyeTracking::Run()
{
    if (!Setup())
        return;

    bool Carry = true;
    while (Carry)
    {
        if(Cam.GetNumOfWindows() +
                Tracker.GetNumOfWindows() +
                Cali.GetNumOfWindows() +
                Img_Proc.GetNumWindows() +
                Calcs.GetNumWindows() == 0)
        {
            //Tracker.ShowWindow();
            Cam.ShowImage();
        }

        Carry = RunCommnad(cvWaitKey(0));

    }

    Cam.StopCapture();
}

bool EyeTracking::RunCommnad(char Com)
{
    switch (Com)
    {
    case 27:
        return false;
        break;
    case '1':
        Img_Proc.ShowCaliWindow();
        break;
    case '2':
        Img_Proc.HideCaliWindow();
        break;
    case 'a':
    case 'A':
        Tracker.ShowWindow();
        break;
    case 's':
    case 'S':
        Tracker.HideWindow();
        break;
    case 'q':
    case 'Q':
        Cam.ShowImage();
        break;
    case 'w':
    case 'W':
        Cam.HideImage();
        break;
    case ' ':
        if (DiffFound & Cali.Calibrating())
        {
            if (!Cali.TakeCaliPoint(CurDiff))
            {
                Calcs.UpdateCalibration(Cali.GetCalibrationPoints(), Cali.GetCalibrationLocations());
            }
        }
        break;
    case 'c':
    case 'C':
        Cali.StartCalibration();
        break;
    case 'n':
    case 'N':
        Calcs.ShowWindow();
        break;
    case 'm':
    case 'M':
        Calcs.HideWindow();
        break;
    }
    return true;
}

void EyeTracking::RunBehind()
{
    if (!Setup())
        return;

    Running = true;
    pthread_create(&bk_Runner, NULL, &bk_Working, (void*) this);
}

void EyeTracking::StopBehind()
{
    if (Running)
    {
        Cam.StopCapture();
        Running = false;
        pthread_exit(NULL);
    }
}

void* EyeTracking::bk_Working(void* ptr)
{
    EyeTracking* This = (EyeTracking*)ptr;
    while (This->Running)
    {
        char c =  cvWaitKey(10);
        switch (c)
        {
        case 27:
            This->Running = false;
            return NULL;
        case '1':
            This->Img_Proc.ShowCaliWindow();
            break;
        case '2':
            This->Img_Proc.HideCaliWindow();
            break;
        case 'a':
        case 'A':
            This->Tracker.ShowWindow();
            break;
        case 's':
        case 'S':
            This->Tracker.HideWindow();
            break;
        case 'q':
        case 'Q':
            This->Cam.ShowImage();
            break;
        case 'w':
        case 'W':
            This->Cam.HideImage();
            break;
        case ' ':
            if (This->DiffFound & This->Cali.Calibrating())
            {
                if (!This->Cali.TakeCaliPoint(This->CurDiff))
                {
                    This->Calcs.UpdateCalibration(This->Cali.GetCalibrationPoints(), This->Cali.GetCalibrationLocations());
                }
            }
            break;
        case 'c':
        case 'C':
            This->Cali.StartCalibration();
            break;
        case 'n':
        case 'N':
            This->Calcs.ShowWindow();
            break;
        case 'm':
        case 'M':
            This->Calcs.HideWindow();
            break;
        }
    }

    return NULL;
}

void EyeTracking::DisplayComands()
{
    printf("Commands:\n");
    printf("1: Show Settings Window\n");
    printf("2: Hide Settings Window\n");
    printf("Q: Show Raw Camera Image\n");
    printf("W: Hide Camera Image\n");
    printf("A: Show Tracker Window\n");
    printf("S: Hide Tracker Window\n");
    printf("C: Calibrate\n");
    printf("%c %c: Take Calibration Point\n", 39, 39);
    printf("N: Show Current Point\n");
    printf("M: Hide Current Point\n");
}

void EyeTracking::UpdatedImage(IplImage* Image, void* ptr)
{
    EyeTracking* This = (EyeTracking*)ptr;
    This->Img_Proc.ProcessImage(Image);
}

void EyeTracking::UpdateProcessed(IplImage* Orig, IplImage* Processed, void* ptr)
{
    EyeTracking* This = (EyeTracking*)ptr;
    This->Tracker.Track(Orig, Processed);
}

void EyeTracking::UpdatedLocations(bool Found, EyeDifferance Diff, void* ptr)
{
    EyeTracking* This = (EyeTracking*)ptr;
    This->CurDiff = Diff;
    This->DiffFound = Found;

    if (Found & !This->Cali.Calibrating())
    {
        This->Calcs.FindPoint(Diff);
    }
}
