#ifndef IMAGEPLAYING_H
#define IMAGEPLAYING_H

#include <opencv/cv.h>
#include <opencv/highgui.h>


class ImagePlaying
{
public:
    ImagePlaying();
    ImagePlaying(int, int);
    virtual ~ImagePlaying();

    void SetNum1(int);
    void SetNum2(int);
    int GetNum1();
    int GetNum2();

    void DoAllProcesses(IplImage*, IplImage*);

    void ConvertToBinary(IplImage*, IplImage*);

    void ContourFinder(IplImage*, IplImage*);

    void ExtendLines(IplImage*, IplImage*);
    void ExtendLines(IplImage*, IplImage*, int);
protected:
private:
    //Functions
    unsigned char Modify(unsigned char);

    //Variables
    int PNum1, PNum2;

    //Constants
    static const int StepSize = 2;
    static const int PassAmount = 2;
};

#endif // IMAGEPLAYING_H
