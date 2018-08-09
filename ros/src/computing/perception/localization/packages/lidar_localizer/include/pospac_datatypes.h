#ifndef POSPAC_DATATYPES_H
#define POSPAC_DATATYPES_H

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Transform.h>

struct pospac_export {
    ros::Time timestamp;
    double distance;
    // x, y, orthometric height
    geometry_msgs::Point position;
    double lat;
    double lon;
    double ellipsoid_height;
    geometry_msgs::Quaternion orientation;
    double east_velocity;
    double north_velocity;
    double up_velocity;
    double east_sd;
    double north_sd;
    double height_sd;
    double roll_sd;
    double pitch_sd;
    double heading_sd;
};

#endif // POSPAC_DATATYPES_H
