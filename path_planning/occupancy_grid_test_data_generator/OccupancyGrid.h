//
//  OccupanyGrid.h
//  image_convert
//
//  Created by Tara Sabbineni on 2/10/19.

#ifndef OccupanyGrid
#define OccupanyGrid

class OccupancyGridInfo
{
public:
    OccupancyGridInfo(unsigned int w, unsigned int h):
    resolution(0), width(w), height(h)
    {
        probabilities.resize(width * height, 0);
    }
    
    void set_probability(int i, int item)
    {
        probabilities[i] = item;
    }

private:

    // all the data we can get directly from the image
    float resolution;
    unsigned int width;
    unsigned int height;

    std::vector<int> probabilities;
};

#endif /* OccupanyGrid_h */
