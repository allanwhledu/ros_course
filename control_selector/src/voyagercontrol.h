#ifndef VOYAGERCONTROL_H
#define VOYAGERCONTROL_H

#include <ros/ros.h>
#include "control.h"

class VoyagerControl : public Control
{
    double min_range;
    bool obstacle;
    double max_vel;
    double max_omega;
public:
    //установка данных лазера
    virtual void setLaserData(const std::vector<float>& data);

    //установка текущей позиции робота - для данного вида управления не требуется - поэтому пустая
    virtual void setRobotPose(double x, double y, double theta) {}

    //получение управления
    virtual void getControl(double& v, double& w) ;

    VoyagerControl(double range = 1.0, double maxv = 0.5, double maxw = 0.5):
        obstacle(false),
        min_range(range),
        max_vel(maxv),
        max_omega(maxw)
    {
        ROS_DEBUG_STREAM("VoyagerControl constructor");
    }

    ~VoyagerControl()
    {
        ROS_DEBUG_STREAM("VoyagerControl destructor");
    }
};

#endif // VOYAGERCONTROL_H
