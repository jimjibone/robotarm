#include "EyeTimers.h"

EyeTimers::EyeTimers()
{
    Run = false;
    Every = (long)10000000000;
}

EyeTimers::EyeTimers(long MillisecDelay)
{
    Run = false;
    Every = MillisecDelay;

	Next = 0L;
}

EyeTimers::EyeTimers(double Freq)
{
    Run = false;
    Every = (long) (1.0 / Freq * 1000.0);

	Next = 0L;
}

EyeTimers::~EyeTimers()
{
    Stop();
}

void EyeTimers::Start(ptr2Func Func, void* SentThrough)
{
    Functions = Func;
    SentObj = SentThrough;
    Run = true;
    SYSTEMTIME time;
    GetSystemTime(&time);
    long millis = (time.wHour * 60 * 60 * 1000) + (time.wMinute * 60 * 1000) + (time.wSecond * 1000) + time.wMilliseconds;
    Next = millis + Every;
    pthread_create(&Bk_thread, NULL, &Running, (void*) this);
}

void EyeTimers::Stop()
{
    bool WasRun = Run;
    Run = false;
    Sleep(100);
	//if (WasRun) pthread_exit(NULL);
}

void* EyeTimers::Running(void* ptr)
{
    EyeTimers* This;
    This = (EyeTimers*) ptr;

    while (This->Run)
    {
        SYSTEMTIME time;
        GetSystemTime(&time);
        long millis = (time.wHour * 60 * 60 * 1000) + (time.wMinute * 60 * 1000) + (time.wSecond * 1000) + time.wMilliseconds;

        if (millis > This->Next)
        {
            This->Next = millis + This->Every;
            This->Functions(This->SentObj);
        }
    }

    return NULL;
}

void EyeTimers::WaitTillNext()
{
    SYSTEMTIME time;
    GetSystemTime(&time);
    long millis = (time.wHour * 60 * 60 * 1000) + (time.wMinute * 60 * 1000) + (time.wSecond * 1000) + time.wMilliseconds;

    while (millis < Next)
    {
        Sleep(Next - millis);
        GetSystemTime(&time);
        millis = (time.wHour * 60 * 60 * 1000) + (time.wMinute * 60 * 1000) + (time.wSecond * 1000) + time.wMilliseconds;
    }

    Next = millis + Every;
}

void EyeTimers::Wait(long millis)
{
    Sleep(millis);
}

void EyeTimers::Wait()
{
    Sleep(Every);
}
