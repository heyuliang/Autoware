#ifndef ACCEL_MODEL_BASE_H_INCLUDED
#define ACCEL_MODEL_BASE_H_INCLUDED

namespace accel_model
{
    class accel_model_base
    {
    public:
        accel_model_base();
        ~accel_model_base();
        virtual double get_acceralation_value();
    };
}
#endif  //ACCEL_MODEL_BASE_H_INCLUDED