#ifndef VEHICLE_MODEL_BASE_H_INCLUDED
#define VEHICLE_MODEL_BASE_H_INCLUDED

#include <autoware_msgs/VehicleCmd.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>

namespace vehicle_model
{
    class vehicle_model_base
    {
        public:
            vehicle_model_base();
            ~vehicle_model_base();
            virtual void update(autoware_msgs::VehicleCmd cmd) = 0;
    };
};
#endif  //VEHICLE_MODEL_BASE_H_INCLUDED