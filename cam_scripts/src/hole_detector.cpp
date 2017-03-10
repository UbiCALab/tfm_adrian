#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <tf/transform_listener.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>
#include <algorithm>
#include <math.h>


ros::Publisher pc_pub;
tf::StampedTransform tf_camera;
tf::TransformListener *listener;
tf::TransformListener *listener_robot;
std::string prj_out;
double obstacle_height_, x_range_, z_min_, camera_height_;
struct pointXY
{
    pointXY(){x = 0; y=0;}
    float x;
    float y;
};
std::vector<pointXY> sample_proj_;


void writeSampleProj()
{
    std::ofstream wfile;
    wfile.open(prj_out.c_str());
    for(std::vector<pointXY>::iterator it = sample_proj_.begin(); it != sample_proj_.end(); ++it)
        wfile << it->x << ";" << it->y << "\n";
    wfile.close();
}
void loadReferenceProj(const char *infile)
{
    ROS_INFO("Loading reference projection...");

    sample_proj_.clear();
    std::stringstream ss(infile);
    std::string line, num;

    if (infile != NULL){
        while(std::getline(ss, line,'\n')){
            std::string::size_type pos, prev = 0;
            pointXY pxy;

            pos = line.find(";");
            pxy.x = atof(line.substr(prev, pos).c_str());
            pxy.y = atof(line.substr(pos+1).c_str());

            sample_proj_.push_back(pxy);
        }
    }

    ROS_INFO("Reference projection load");
}

// Get the position of the ground projection of a "pixel" given its index
geometry_msgs::Point getGroundProjectionHeight(int idx, double height)
{
    // Use sample_proj_
    //ROS_INFO("Nan found. Idx: %u, x: %f, y %f. Size sample %u", idx, sample_proj_[idx].x, sample_proj_[idx].y, sample_proj_.size());
    geometry_msgs::Point out;
    float x_proj = sample_proj_[idx].x;
    if (x_proj < x_range_){
        out.x = x_proj;
        out.y = sample_proj_[idx].y;
        out.z = height;
    }else{
        out.x = 0;    out.y = 0;    out.z = 0;
    }

    return out;
}
geometry_msgs::Point getGroundProjection(int idx)
{
    return getGroundProjectionHeight(idx, obstacle_height_);
}

// Deduce pointClout2 structure
void getStructure(const sensor_msgs::PointCloud2& pc)
{
    ROS_INFO("Getting reference projection...");
    sensor_msgs::PointCloud2ConstIterator<float> x_it(pc, "x");
    sample_proj_.clear();
    pointXY prev_xy;

    for(; x_it != x_it.end(); ++x_it){
        // Construct sample projection
        pointXY xyp;
        if (std::isnan(x_it[0]) || std::isnan(x_it[1]) || std::isnan(x_it[2])){
            xyp.x = prev_xy.x;
            xyp.y = prev_xy.y;
        }else{
            xyp.x = x_it[0];
            xyp.y = x_it[1];
            prev_xy = xyp;
        }
        //ROS_INFO("Pushing back to sample_proj_: (%f, %f)", xyp.x, xyp.y);
        sample_proj_.push_back(xyp);
    }
    ROS_INFO("Reference projection stored");
}

void pointCloud2Callback(const sensor_msgs::PointCloud2ConstPtr& pc)
{
    // Transform pointcloud
    sensor_msgs::PointCloud2 cloud_out;
    pcl_ros::transformPointCloud("/base_footprint", tf_camera, *pc, cloud_out);

    // Check if pointCloud is structured
    if (sample_proj_.size()==0){
        getStructure(cloud_out);

        if (!prj_out.empty()){
            ROS_INFO("Writing reference projection to %s", prj_out.c_str());
            writeSampleProj();
        }
    }


    // Iterate over the poinCloud2
    //ROS_INFO("PC size: %u, width: %f, height: %f", pc->data.size(), pc->row_step, pc->point_step);
    int i = 0;
    double frac;
    int nans = 0, obs = 0, common = 0;
    for(sensor_msgs::PointCloud2Iterator<float> iter_x(cloud_out, "x"); iter_x != iter_x.end(); ++iter_x){
        if(iter_x[2] < z_min_){
            frac = camera_height_ / (camera_height_ - iter_x[2]);
            iter_x[0] = iter_x[0]*frac;
            iter_x[1] = iter_x[1]*frac;
            iter_x[2] = obstacle_height_;
            obs++;
        }else if(std::isnan(iter_x[0]) && std::isnan(iter_x[1]) && std::isnan(iter_x[2])){
            geometry_msgs::Point p = getGroundProjection(i);
            iter_x[0] = p.x;
            iter_x[1] = p.y;
            iter_x[2] = p.z;
            nans++;
        }else{
            for(int j = 0; j<3; j++)
                iter_x[j] = 0;
            common++;
        }
        i++;
    }
    //ROS_INFO("Found %u common, %u nans and %u obstacles", common, nans, obs);

    ROS_INFO_ONCE("Streaming clouds.");
    pc_pub.publish(cloud_out);
}

int main(int argc, char **argv)
{
    // Init node
    ros::init(argc, argv, "hole_detector");
    ros::NodeHandle n("hole_detector");

    //  Publishers transform listeners and subscribers
    pc_pub = n.advertise<sensor_msgs::PointCloud2>("/camera_corrected/depth/points", 1);
    listener = new tf::TransformListener();
    listener_robot = new tf::TransformListener();

    // Waiting period to allow listeners to store tfs
    ros::Duration(0.2).sleep();

    ros::Subscriber sub_top = n.subscribe("/camera_front_cam/depth/points", 1, pointCloud2Callback);

    // Getting parameters
    std::string prj_in;
    n.param<double>("obstacle_height", obstacle_height_, 0.2);
    n.param<double>("obstacle_distance", x_range_, 3.0);
    n.param<double>("minimum_height", z_min_, -0.01);
    n.param<std::string>("projection_out", prj_out, "");
    ROS_INFO("Chosen parameters: Obstacle height: %f, Obstacle distance: %f, Minimum height: %f, fout: %s", obstacle_height_, x_range_, z_min_, prj_out.c_str());

    if(n.getParam("projection_in", prj_in)){
        loadReferenceProj(prj_in.c_str());
    }else{
        ROS_WARN("Reference projection not specified. The first pointcloud sample will be taken as reference.");
    }


    // Getting static transform
    ROS_INFO("hole_detector: Waiting for transform");
    tf::TransformListener cam_listener;
    bool found_tf = 0;
    while(!found_tf){
	try{
	    cam_listener.lookupTransform("/base_footprint", "/camera_front_cam_depth_optical_frame", ros::Time(0), tf_camera);
	    found_tf = true;
	}
	catch (tf::TransformException ex){
	    //ROS_ERROR("%s",ex.what());
	    ros::Duration(1.0).sleep();
	}
    }
    ROS_INFO("Camera info: tf_parent: %s, tf_child: %s, height: %f", tf_camera.frame_id_.c_str(),
									tf_camera.child_frame_id_.c_str(),
									tf_camera.getOrigin().z());
    camera_height_ = tf_camera.getOrigin().z();




    ros::spin();
    return 0;
}

