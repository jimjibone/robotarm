#include "ImagePlaying.h"

ImagePlaying::ImagePlaying()
{
    PNum1 = 40;
    PNum2 = 100;
}

ImagePlaying::ImagePlaying(int Num1, int Num2)
{
    PNum1 = Num1;
    PNum2 = Num2;
}

ImagePlaying::~ImagePlaying()
{

}


void ImagePlaying::SetNum1(int Num)
{
    PNum1 = Num;
}

void ImagePlaying::SetNum2(int Num)
{
    PNum2 = Num;
}

int ImagePlaying::GetNum1()
{
    return PNum1;
}

int ImagePlaying::GetNum2()
{
    return PNum2;
}


void ImagePlaying::DoAllProcesses(IplImage* ScrImage, IplImage* DestImage)
{
    IplImage* Img1 = cvCreateImage(cvGetSize(ScrImage), 8, 1);
    ConvertToBinary(ScrImage, Img1);
    IplImage* Img2 = cvCreateImage(cvGetSize(ScrImage), 8, 1);
    ContourFinder(Img1, Img2);
    cvFree(&Img1);
    ExtendLines(Img2, DestImage, 1);
    cvFree(&Img2);
}


void ImagePlaying::ConvertToBinary(IplImage* ScrImage, IplImage* DestImage)
{//Searches each point in an image and decides what binary value to assign it based on inputs
    for (int X = 0; X < ScrImage->width; X++)
    {
        for (int Y = 0; Y < ScrImage->height; Y++)
        {
            int P = X + Y * ScrImage->width;
            DestImage->imageData[P] = Modify(ScrImage->imageData[P]);
        }
    }
}

unsigned char ImagePlaying::Modify(unsigned char Num)
{//3 Point return, allows for slightly more accurate contouring it seems
    if(Num < PNum1)
    {
        return 0;
    }
    else if (Num < PNum2)
    {
        return 127;
    }
    else
    {
        return 255;
    }
}


void ImagePlaying::ContourFinder(IplImage* ScrImage, IplImage* DestImage)
{//No idea how efficent this is compared to some but seems to work well
    for (int X = StepSize; X < ScrImage->width - StepSize; X++)
    {
        for (int Y = StepSize; Y < ScrImage->height - StepSize; Y++)
        {
            int Num = 0; //Test value, if enough points pass that is considered a valid point
            int P = X + Y * ScrImage->width;
            char CheckAgainst = ScrImage->imageData[P]; //Take value of the point in question to see if there are differances
            for (int x = X - StepSize; x < X + StepSize; x++)
            {//Dont just check points next to for accuracy
                for (int y = Y - StepSize; y < Y + StepSize; y++)
                {
                    if (x != X && Y != y)
                    {//Dont check
                        if (CheckAgainst != ScrImage->imageData[x + y *ScrImage->width]) //If value changes, add a value to tet variable
                        {
                            Num++;

                            if (Num > PassAmount)
                            {
                                x = X + StepSize;
                                break;
                            }
                        }
                    }
                }
            }

            if (Num > PassAmount)
            {
                DestImage->imageData[P] = (char)-1;
            }
            else
            {
                DestImage->imageData[P] = (char)0;
            }
        }
    }
}


void ImagePlaying::ExtendLines(IplImage* ScrImage, IplImage* DestImage)
{//Extneds lines using default size
    ExtendLines(ScrImage, DestImage, StepSize);
}

void ImagePlaying::ExtendLines(IplImage* ScrImage, IplImage* DestImage, int Width)
{//Extends lines around contours to make processing more accurate
    for (int X = Width; X < ScrImage->width - Width ; X++)
    {
        for (int Y = Width; Y < ScrImage->height - Width; Y++)
        {
            int P = X + Y * ScrImage->width;
            char data = ScrImage->imageData[P];
            if (data == -1)
            {
                DestImage->imageData[P] = (char)-1;
            }
            else
            {
                DestImage->imageData[P] = 0;
                for (int x = X - Width; x < X + Width; x++)
                {
                    for (int y = Y - Width; y < Y + Width; y++)
                    {
                        if (ScrImage->imageData[x + y * ScrImage->width] == -1)
                        {
                            DestImage->imageData[P] = (char)-1;
                            x = X + Width;
                            break;
                        }
                    }
                }
            }
        }
    }
}
