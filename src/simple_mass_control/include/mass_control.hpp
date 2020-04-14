#ifndef MASS_CONTROL_HPP
#define MASS_CONTROL_HPP

#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/gazebo_client.hh>
#include <gazebo_msgs/SetModelState.h>
#include <gazebo_msgs/GetModelState.h>
#include <gazebo_msgs/ApplyBodyWrench.h>
#include <gazebo_msgs/GetPhysicsProperties.h>
#include <gazebo_msgs/GetLinkProperties.h>

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Wrench.h>

#include <iostream>


class MassControl
{
public:
    MassControl(ros::NodeHandle *n);
    ~MassControl(void);
private:
    void update_parameters(ros::NodeHandle *n);
    double m = 1.0;
    double g = 9.8;
};

#endif
