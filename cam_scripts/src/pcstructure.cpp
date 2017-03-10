#include <cam_scripts/pcstructure.h>

#include <sensor_msgs/point_cloud2_iterator.h>
#include <algorithm>
#include <math.h>

using namespace std;


void idx::operator=(const idx id2)
{
    x = id2.x;
    y = id2.y;
}
bool idx::operator==(const idx id2)
{
    return id2.x == x && id2.y == y;
}


template <typename T>
T median(vector<T> &v)
{
    size_t n = v.size() / 2;
    nth_element(v.begin(), v.begin()+n, v.end());
    return v[n];
}

PCStructure::PCStructure()
{
    verbosity = 1;
    hasStructure = false;
}


void PCStructure::deduceStructure(const sensor_msgs::PointCloud2& pc)
{
    if(verbosity > 0)
	ROS_INFO("Deducing point cloud structure");

    vector<unsigned int> lengths;
    sensor_msgs::PointCloud2ConstIterator<float> x_it(pc, "x");
    unsigned int prev_idx = 0, idx = 0;
    float prev_x = x_it[0], x;
    float threshold = 0.1;

    for(; x_it != x_it.end(); ++x_it){
	x = x_it[0];
	if (abs(x - prev_x) > threshold){
	    lengths.push_back(idx - prev_idx);
	    prev_idx = idx;
	}

	++idx;
	prev_x = x;

    }

    width = median(lengths);
    height = idx / width;


    ROS_INFO("Idx is %u", idx);
    if (verbosity > 0)
    	printStructure();


    hasStructure = true;
}

void PCStructure::printStructure()
{
    ROS_INFO("PCStructure: The point cloud has width: %u, height: %u, total number of points: %u", width, height, width * height);
}


idx PCStructure::getIndex(unsigned int i)
{
    idx out;
    out.x = i / width;
    out.y = i % width;
    return out;
}
