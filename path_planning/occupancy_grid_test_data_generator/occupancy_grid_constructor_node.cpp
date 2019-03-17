#include <opencv2/opencv.hpp>
#include "OccupancyGrid.h"
#include <iostream>
#include <fstream>

using namespace cv;
using namespace std;

OccupancyGridInfo OccupancyGridInfo::fill_probabilities(int argc, char **argv)
{
    // Read the image file
    Mat image;
    image = imread(argv[1], CV_LOAD_IMAGE_COLOR);
    
    // Make sure we can open the file
    if(!image.data)
    {
        cout << "Could not open or find the image" << endl;
        exit(-1);
    }
    
    Mat HSV;
    cvtColor(image, HSV, CV_BGR2HSV);
    
    // Create a matrix with only the value integers from the image
    Mat hsv_values[3];
    split(HSV, hsv_values);
    Mat v = hsv_values[2];
    
    // Initialize the grid object with the image's width and height
    OccupancyGridInfo grid((unsigned int)v.size().height,
                           (unsigned int)v.size().width);
    
    // Fill the probability vector
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
    return grid;
}

int main(int argc, char** argv)
{
    OccupancyGridInfo grid;
    grid = grid.fill_probabilities(argc, argv);

    ofstream outFile("occupancyGrid.yaml");
    outFile << "Header:" << endl;
    outFile << " - seq: 0" << endl;
    outFile << " - timeStamp: 0" << endl;
    outFile << " - frameID: 0" << endl;

    outFile << "MapMetaData:" << endl;
    outFile << " - mapLoadTime: 0" << endl;
    outFile << " - resolution: " << grid.getResolution() << endl;
    outFile << " - width: " << grid.getWidth() << endl;
    outFile << " - height: " << grid.getHeight() << endl;


    outFile << "Data:";
    vector<int> prob = grid.getProbabilities();
    for(int i = 0; i < (int) prob.size(); i++) {
        outFile << " - " << prob.at(i) << endl;
    }

    return 0;
}
