#include <vehicle_dynamics_lib/accel_model/accel_model_base.h>
#include <pluginlib/class_loader.h>

namespace accel_model
{
    class simple_delay_model : public accel_model_base
    {
    public:
        simple_delay_model();
        ~simple_delay_model();
        double get_acceralation_value();
    };
};