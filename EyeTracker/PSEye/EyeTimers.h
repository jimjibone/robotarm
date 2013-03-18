#ifndef EYETIMERS_H
#define EYETIMERS_H

#include <iostream>
#include <windows.h>
#include <pthread.h>

typedef void (*ptr2Func)(void* ptr);

class EyeTimers
{
    public:
        EyeTimers();
        EyeTimers(long MillisecDelay);
        EyeTimers(double Freq);
        virtual ~EyeTimers();

        void Start(ptr2Func Func, void* SentThrough);
        void Stop();

        void WaitTillNext();

        void Wait();
        void Wait(long millis);
    protected:
    private:
        ptr2Func Functions;
        void* SentObj;

        pthread_t Bk_thread;
        static void* Running(void*);

        bool Run;

        long Every, Next;
};

#endif // EYETIMERS_H
