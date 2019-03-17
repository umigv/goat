//  OccupanyGrid.h
//  image_convert
//
//  Created by Tara Sabbineni on 2/10/19.

#ifndef OccupanyGrid
#define OccupanyGrid

#include <vector>

class OccupancyGridInfo
{
public:
    OccupancyGridInfo():
    resolution(1), width(0), height(0)
    {
    }
    
    OccupancyGridInfo(unsigned int w, unsigned int h):
    resolution(1), width(w), height(h)
    {
        probabilities.resize(width * height, 0);
    }
    
    float getResolution() {
        return resolution;
    }

    unsigned int getWidth() {
        return width;
    }
    unsigned int getHeight() {
        return height;
    }

    std::vector<int> getProbabilities() {
        return probabilities;
    }

    void set_probability(int i, int item)
    {
        probabilities[i] = item;
    }
    
    OccupancyGridInfo fill_probabilities(int argc, char** argv);

private:

    // all the data we can get directly from the image itself
    float resolution;
    unsigned int width;
    unsigned int height;

    std::vector<int> probabilities;
};

#endif /* OccupanyGrid_h */
