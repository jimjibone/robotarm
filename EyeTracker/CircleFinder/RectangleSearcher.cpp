#include "RectangleSearcher.h"

RectangleSearcher::RectangleSearcher()
{
    for (int i = 0; i < Length; i++)
    {
        Rects[i] = EyeRectangle();
    }
}

RectangleSearcher::RectangleSearcher(int Width, int Height)
{
    int W5 = Width / 20;
    int W20 =  W5 * 4;
    int W15 = W5 * 3;
    int W50 = W5 * 10;

    int H5 = Height / 20;
    int H20 = H5 * 4;
    int H15 = H5 * 3;
    int H50 = H5 * 10;

    Rects[0] = EyeRectangle(W15 * 2, H15 * 2, W20 * 2, H20 * 2);
    Rects[1] = EyeRectangle(W15, H15, W15, H20 * 2 + 2 * H15);
    Rects[2] = EyeRectangle(W50 + W20, H15, W15, H20 * 2 + 2 * H15);
    Rects[3] = EyeRectangle(W15 * 2, H15, W20 * 2, H15);
    Rects[4] = EyeRectangle(W15 * 2, H50 + H20, W20 * 2, H15);
    Rects[5] = EyeRectangle(0, 0, W15, H50);
    Rects[6] = EyeRectangle(W50 + W20 + W15, 0, Width - W50 - W20 - W15, H50);
    Rects[7] = EyeRectangle(0, H50, W15, Height - H50);
    Rects[8] = EyeRectangle(W50 + W20 + W15, H50, Width - W50 - W20 - W15, Height - H50);
    Rects[9] = EyeRectangle(W15, 0, W15  + W20, H15);
    Rects[10] = EyeRectangle(W50, 0, W20 + W15, H15);
    Rects[11] = EyeRectangle(W15, H50 + H20 + H15, W15 + W20, Height - (H50 + H20 + H15));
    Rects[12] = EyeRectangle(W50, H50 + H20 + H15, W15 + W20, Height - (H50 + H20 + H15));
}


RectangleSearcher::~RectangleSearcher()
{

}
