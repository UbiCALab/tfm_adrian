## Package cam_scripts

This package provides several nodes which are tools to add functionality to the camera(s).

The nodes are:
#### cloud_merge
Tool for merging two different `sensor_msgs::PointCloud2` in a single message. Running the node will:
- Subscribe to `/camera_top_cam/depth/points` and `/camera_front_cam/depth/points`.
- Publish the merged cloud to `/camera_merged/depth/points`.

#### hole_detector
Detects holes and NANs from a point cloud and publishes them. There should be a *calibration* stage in which the robot camera is facing the ground with no obstacles in between. If no explicit calibration file is used, the first point cloud is taken as reference, which may cause error. If an output calibration file is specified, the calibration used is stored in such file.  
Hence, the recommended way to use this package is:  

1. Move the robot such that its camera *sees no obstacle*.
2. Take a reference pointcloud with `rosrun cam_scripts hole_detector projection_out:=myfile` and killing the node when the projection has been stored.
3. To load the projection use `rosrun cam_scripts hole_detector projection_in:=myfile`.

See [hole_detector.launch](https://github.com/UbiCALab/advanrobot_devel/blob/master/cam_scripts/launch/hole_detector.launch)
for an example file.

#### pc_trimmer_node
Subscribes to a point cloud message and publishes a trimmed version of it. The trimming function can be defined. To launch the node with the default trimming function (T-shape trimming) simply run the node:
```sh
rosrun cam_sctipts pc_trimmer_node
```
Which will subscribe to `/camera_front_cam/depth/points` and publish to `/camera_trimmed/depth/points`.

#### range_publisher
Publishes a [sensor_msg::Range](http://docs.ros.org/api/sensor_msgs/html/msg/Range.html) for visualisation purposes (camera range).  
Published topics:

- Camera_front:
  + topic: *front_camera_range*
  + frame: */camera_front_cam_depth_optical_frame*
- Camera_top:
  + topic: *top_camera_range*
  + frame: */camera_top_cam_depth_optical_frame*
