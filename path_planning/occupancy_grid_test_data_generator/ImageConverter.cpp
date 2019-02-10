#include <opencv2/opencv.hpp>
#include "OccupancyGrid.h"

using namespace cv;
using namespace std;

int main(int argc, char** argv)
{
    // Read the image file
    Mat image;
    image = imread(argv[1], CV_LOAD_IMAGE_COLOR);   // Read the file
    
    if(!image.data)                              // Check for invalid input
    {
        cout <<  "Could not open or find the image" << std::endl;
        return -1;
    }
    
    Mat HSV;
    
    cvtColor(image, HSV, CV_BGR2HSV);
    
    Mat hsv_values[3];
    split(HSV, hsv_values);
    Mat v = hsv_values[2];
    
    OccupancyGridInfo grid((unsigned int)v.size().height,
                           (unsigned int)v.size().width);
    
    for(int row = 0; row < v.rows; row++)
    {
        int i = 0;
        uchar* p = v.ptr(row);
        for(int col = 0; col < v.cols; col++)
        {
            grid.set_probability(i, (int)(*p)*(100.0/255.0));
            p++;
            i++;
        }
    }
    
    namedWindow("value", CV_WINDOW_AUTOSIZE);
    imshow("value", v);
    
    namedWindow("image", CV_WINDOW_AUTOSIZE);
    imshow("image", image);
    
    cv::waitKey(0);
    
    return 0;
}
