#ifndef IMAGEPLAYING_H
#define IMAGEPLAYING_H

#include <opencv/cv.h>
#include <opencv/highgui.h>


class ImagePlaying
{
public:
    ImagePlaying();
    ImagePlaying(int Num1, int Num2);
    virtual ~ImagePlaying();

    void SetNum1(int Num);
    void SetNum2(int Num);
    int GetNum1();
    int GetNum2();

    void DoAllProcesses(IplImage* ScrImage, IplImage* DestImage);

    void ConvertToBinary(IplImage* ScrImage, IplImage* DestImage);

    void ContourFinder(IplImage* ScrImage, IplImage* DestImage);

    void ExtendLines(IplImage* ScrImage, IplImage* DestImage);
    void ExtendLines(IplImage* ScrImage, IplImage* DestImage, int Width);
protected:
private:
    //Functions
    unsigned char Modify(unsigned char Num);

    //Variables
    int PNum1, PNum2;

    //Constants
    static const int StepSize = 2;
    static const int PassAmount = 2;
};

#endif // IMAGEPLAYING_H
