#include <cam_scripts/pc_trimmer.h>
#include <sensor_msgs/point_cloud2_iterator.h>

using namespace std;

AutoTrimFunction::AutoTrimFunction(const vector<int>& x, const vector<int>& y)
{
    if (x.size() != y.size()){
    	ROS_ERROR("AutoTrimFunction: entering vectors have not the same size. %lu != %lu", x.size(), y.size());
    	return;
    }


    vector<idx> to_trim_in(x.size());

    for (size_t i = 0; i < x.size(); ++i) {
    	to_trim_in[i] = idx(x[i], y[i]);
    }


    to_trim = to_trim_in;

}


bool AutoTrimFunction::operator()(unsigned int x, unsigned int y)
{
    idx coords(x, y);

    vector<idx>::iterator it = find(to_trim.begin(), to_trim.end(), coords);

    return it == to_trim.end();
}


void PCTrimmer::trim(sensor_msgs::PointCloud2& pc)
{
    if(!pc_structure.hasStruct()){
    	ROS_WARN("PCTrimmer: There is no structure information. It will de deduced, this may cause problems if the camera does not see completely the ground");
    	pc_structure.deduceStructure(pc);
    }

    sensor_msgs::PointCloud2Iterator<float> x_it(pc, "x");
    int changed = 0, no_changed = 0;
    int i = 0;
    for(; x_it != x_it.end(); ++x_it){
	idx coords = pc_structure.getIndex(i);
	//ROS_INFO("Coords (%d, %d). Result: %d", coords.x, coords.y, trim_fun(coords.x, coords.y));
	if(trim_fun(coords.x, coords.y)){
	    x_it[0] = 0;
	    x_it[1] = 0;
	    x_it[2] = 0;
	    changed++;
	}
	else
	    no_changed++;
	++i;
    }

    ROS_INFO("PCTrimmerNode: There have been %d changes and %d no changes", changed, no_changed);
}
