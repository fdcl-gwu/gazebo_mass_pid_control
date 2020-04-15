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

#include <iostream>

#include "mass_control.hpp"

ignition::math::Vector3d MassControl::x = ignition::math::Vector3d::Zero;
ignition::math::Vector3d MassControl::v = ignition::math::Vector3d::Zero;

int main(int argc, char **argv)
{
    ROS_INFO("Starting simple mass control");
    gazebo::client::setup(argc, argv);
    ros::init(argc, argv, "simple_mass_control");

    gazebo::transport::NodePtr node(new gazebo::transport::Node());
    node->Init();

    ros::NodeHandle n;

    MassControl controller(&n);

    // ros::Timer timer_uav_control = n.createTimer( \
    //     ros::Duration(0.01), std::bind(&UavControl::update, Uav)
    // );
    ros::spin();

    return 0;
}
