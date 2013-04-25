#ifndef EYETIMERS_H
#define EYETIMERS_H

#include <iostream>
#include <windows.h>
#include <pthread.h>

typedef void (*ptr2Func)(void*);

class EyeTimers
{
public:
    EyeTimers();
    EyeTimers(long);
    EyeTimers(double);
    virtual ~EyeTimers();

    void Start(ptr2Func, void*);
    void Stop();

    void WaitTillNext();

    void Wait();
    void Wait(long);
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
