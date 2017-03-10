#ifndef CAM_SCRIPTS_PCSTRUCTURE_H_
#define CAM_SCRIPTS_PCSTRUCTURE_H_

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>


struct idx
{
    idx() : x(0), y(0) {}
    idx(unsigned int x_in, unsigned int y_in) : x(x_in), y(y_in) {}

    bool operator==(const idx);
    void  operator=(const idx);
    unsigned int x, y;
};


class PCStructure
{
public:
    PCStructure();

    void deduceStructure(const sensor_msgs::PointCloud2&);
    void printStructure();
    idx getIndex(unsigned int i);
    bool hasStruct() { return hasStructure; }

    unsigned int getWidth() {return width;}
    unsigned int getHeight() {return height;}
private:
    unsigned int width, height, verbosity;
    bool hasStructure;
};


#endif  // CAM_SCRIPTS_PCSTRUCTURE_H_
