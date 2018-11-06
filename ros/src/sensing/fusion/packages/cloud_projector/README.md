# Cloud Projector
This node will project a Pointcloud to the image space as defined in the TF tree using the Camera Intrinsics.

### Requirements

1. Camera intrinsics
1. Camera-LiDAR extrinsics
1. PointCloud 


### How to launch

* From a sourced terminal:

`ROS_NAMESPACE=/camera_ns rosrun cloud_projector cloud_projector`

* From Runtime Manager:

Soon

### Parameters

Launch file available parameters:

|Parameter| Type| Description|Default|
----------|-----|--------|---|
|`points_src`|*String* |Name of the PointCloud topic to subscribe.|Default `points_raw`|
|`camera_info_src`|*String*|Name of the CameraInfo topic that contains the intrinsic matrix for the Image.|`camera_info`|

### Subscriptions/Publications


```
Publications: 
 * /image_depth [sensor_msgs/Image]

Subscriptions: 
 * /points_raw [sensor_msgs/PointCloud type]
 * /tf [tf2_msgs/TFMessage]
```
