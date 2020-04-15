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
#include <ignition/math/Vector3.hh>
#include "std_msgs/String.h"

#include "simple_mass_control/states.h"

#include <iostream>

// typedef Vec3 ignition::math::Vector3d;

class MassControl
{
public:
    MassControl(ros::NodeHandle *n);
    ~MassControl(void);

    void update_states(void);
    void print_msg(void);

    static ignition::math::Vector3d x;
    static ignition::math::Vector3d v;

private:
    void update_parameters(ros::NodeHandle *n);
    void set_initial_pose(ros::NodeHandle *n);
    static void update_states(const simple_mass_control::states::ConstPtr &msg);
    static void msg_to_vector(
        geometry_msgs::Vector3 msg_vec, ignition::math::Vector3d &vec
    );
 

    ros::Subscriber sub_state;

    double m = 1.0;
    double g = 9.8;

    ignition::math::Vector3d xd = ignition::math::Vector3d::Zero;
    ignition::math::Vector3d vd = ignition::math::Vector3d::Zero;
};

#endif
