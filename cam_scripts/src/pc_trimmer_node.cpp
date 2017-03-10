#include <cam_scripts/pc_trimmer.h>
#include <cam_scripts/pcstructure.h>


ros::Publisher pub;
PCStructure pc_structure;


bool trimFun(unsigned int x, unsigned int y)
{
    unsigned int offset = 60;
    unsigned int width = pc_structure.getWidth(), height = pc_structure.getHeight();
    bool bottom = x < offset;
    bool top = x > height - offset * 5;
    bool right = y < offset;
    bool left = y > width - offset;

    bool condition = (top && right) ||
		    (top && left) ||
		    (bottom && right) ||
		    (bottom && left);

    return condition;
}


void pcCallback(const sensor_msgs::PointCloud2& pc_in)
{
    ROS_INFO_ONCE("Received first PointCloud2");

    if (!pc_structure.hasStruct()){
    	pc_structure.deduceStructure(pc_in);
    }

    sensor_msgs::PointCloud2 pc_out(pc_in);

    PCTrimmer trimmer(pc_structure);
    trimmer.setTrimmingFunction(trimFun);

    trimmer.trim(pc_out);

    pub.publish(pc_out);

}

int main(int argc, char **argv)
{
    // Init node
    ros::init(argc, argv, "pc_trimmer");
    ros::NodeHandle n;

    // Publishers ans subscribers
    pub = n.advertise<sensor_msgs::PointCloud2>("/camera_trimmed/depth/points", 1);
    ros::Subscriber sub = n.subscribe("/camera_front_cam/depth/points", 1, pcCallback);

    ros::Rate rate(1);
    while(ros::ok()){
	ros::spinOnce();
	rate.sleep();
    }

}
