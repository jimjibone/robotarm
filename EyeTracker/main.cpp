#include <iostream>
#include "EyeTracking.h"

using namespace std;

int main()
{
    EyeTracking Tracker;

	//When running inline with main
    Tracker.Run();

	//When running in seperate thread
	Tracker.RunBehind();

	while(1)
	{
		//Allows additional programs to be run as well
		Tracker.RunCommnad('a');//Inputs from main thread can be made to eyetracker thread
	}

	Tracker.StopBehind();
    return 0;
}
