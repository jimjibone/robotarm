#ifndef RECTANGLESEARCHER_H
#define RECTANGLESEARCHER_H

#include "Variables/Rectangle.h"
#include <iostream>

//Container for modified searcher
//Searches middle of the rectangle first and moves out
class RectangleSearcher
{
public:
    RectangleSearcher();
    RectangleSearcher(int Width, int Height);
    virtual ~RectangleSearcher();

    EyeRectangle Rects[13];
    static const int Length = 13;
protected:
private:
};

#endif // RECTANGLESEARCHER_H
