#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <pcl/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>

ros::Publisher pc_pub;
tf::TransformListener *listener;
sensor_msgs::PointCloud2 output, output1, output2;
pcl::PointCloud<pcl::PointXYZ> output_pcl, output1_pcl, output2_pcl;


void cloud_cb1(const sensor_msgs::PointCloud2ConstPtr& in1)
{
    ROS_INFO("cb1: Recieved new cloud");
    listener->waitForTransform("/base_footprint", (*in1).header.frame_id, (*in1).header.stamp, ros::Duration(5.0));
    pcl_ros::transformPointCloud("/base_footprint", *in1, output1, *listener);
    pcl::fromROSMsg(output1, output1_pcl);
    output_pcl = output1_pcl;
    output_pcl += output2_pcl;
    pcl::toROSMsg(output_pcl, output);
    pc_pub.publish(output);
    ROS_INFO("cb1: Published point cloud");
}

void cloud_cb2(const sensor_msgs::PointCloud2ConstPtr& in2)
{
    ROS_INFO("cb2: Recieved new cloud");
    listener->waitForTransform("/base_footprint", (*in2).header.frame_id, (*in2).header.stamp, ros::Duration(5.0));
    pcl_ros::transformPointCloud("/base_footprint", *in2, output2, *listener);
    pcl::fromROSMsg(output2, output2_pcl);
    output_pcl = output2_pcl;
    output_pcl += output1_pcl;
    pcl::toROSMsg(output_pcl, output);
    pc_pub.publish(output);
    ROS_INFO("cb2: Published point cloud");
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "cloud_merge");
    ros::NodeHandle n;

    listener = new tf::TransformListener();
    ros::Subscriber sub_top = n.subscribe("/camera_top_cam/depth/points", 1, cloud_cb1);
    ros::Subscriber sub_front = n.subscribe("/camera_front_cam/depth/points", 1, cloud_cb2);

    pc_pub = n.advertise<sensor_msgs::PointCloud2>("/camera_merged/depth/points", 1);

    ros::spin();
    return 0;
}
