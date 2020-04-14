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

#include "mass_control.hpp"

int main(int argc, char **argv)
{
    ROS_INFO("Starting simple mass control");
    gazebo::client::setup(argc, argv);
    ros::init(argc, argv, "simple_mass_control");

    gazebo::transport::NodePtr node(new gazebo::transport::Node());
    node->Init();

    ros::NodeHandle n;
    geometry_msgs::Pose start_pose;

    MassControl controller(&n);

    // gazebo_msgs::GetPhysicsPropertiesRequest world_physics_request;
    // gazebo_msgs::GetPhysicsPropertiesResponse world_physics;
    // ros::ServiceClient client_world_physics \
    //     = n.serviceClient<gazebo_msgs::GetPhysicsProperties>( \
    //     "/gazebo/get_physics_properties"
    // );
    // client_world_physics.call(world_physics_request, world_physics);
    // const double g = -world_physics.gravity.z;
    //
    // gazebo_msgs::GetLinkPropertiesRequest mass_property_request;
    // gazebo_msgs::GetLinkPropertiesResponse mass_properties;
    //
    // ros::ServiceClient client_mass_properties \
    //     = n.serviceClient<gazebo_msgs::GetLinkProperties>( \
    //     "/gazebo/get_link_properties"
    // );
    // mass_property_request.link_name = (std::string) "unit_box::link";
    // client_mass_properties.call(mass_property_request, mass_properties);
    // double m = mass_properties.mass;
    //
    // std::cout << m << std::endl;
    // ros::spinOnce();
    //
    // start_pose.position.x = 0.0;
    // start_pose.position.y = 0.0;
    // start_pose.position.z = 0.5;
    // start_pose.orientation.x = 0.0;
    // start_pose.orientation.y = 0.0;
    // start_pose.orientation.z = 0.0;
    // start_pose.orientation.w = 0.0;
    //
    //
    // gazebo_msgs::ModelState modelstate;
    // modelstate.model_name = (std::string) "uav";
    // modelstate.reference_frame = (std::string) "world";
    // modelstate.pose = start_pose;
    //
    // gazebo_msgs::ApplyBodyWrenchRequest modelphysics;
    // // modelstate.wrench = wrench;
    //
    // double freq = 1000.0;
    // ros::Rate loop_rate(freq);
    // ros::ServiceClient client = n.serviceClient<gazebo_msgs::SetModelState>( \
    //         "/gazebo/set_model_state"
    // );
    // ros::ServiceClient fm_client = n.serviceClient<gazebo_msgs::ApplyBodyWrench>( \
    //         "/gazebo/apply_body_wrench"
    // );
    // ros::ServiceClient fm0_client = n.serviceClient<gazebo_msgs::ApplyBodyWrench>( \
    //         "/gazebo/clear_body_wrench"
    // );
    //
    // gazebo_msgs::SetModelState setmodelstate;
    // setmodelstate.request.model_state = modelstate;
    // client.call(setmodelstate);
    // ros::spinOnce();
    // loop_rate.sleep();
    //
    //
    // // modelphysics.body_name = (std::string) "uav";
    // // modelphysics.reference_frame = (std::string) "world";
    //
    //
    // geometry_msgs::Wrench delta_wrench, prev_wrench;
    // prev_wrench.force.z = 0.0;
    // delta_wrench.force.z = 0.0;
    //
    // UavControl Uav(&n);
    // Uav.update_parameters(m, g);
    //
    // int ii = 0;
    // // while(ros::ok())
    // // {
    //     // start_pose.position.x += 0.001;
    //     // start_pose.position.z += 0.01;
    //     // modelstate.pose = start_pose;
    //     // setmodelstate.request.model_state = modelstate;
    //     // client.call(setmodelstate);
    //
    //
    //     // wrench.force.z = (double)(m * g);
    //     // setwrenchstate.request.start_time = ros::Time(0.0);
    //     // setwrenchstate.request.duration = ros::Duration(-1);
    //     // delta_wrench.force.z = wrench.force.z - prev_wrench.force.z;
    //     // setwrenchstate.request.wrench = delta_wrench;
    //     // int ret = fm_client.call(setwrenchstate);
    //     //
    //     // prev_wrench.force.z = wrench.force.z;
    //     //
    //     // if (ii < 3)
    //     // {
    //     //     std::cout << setwrenchstate.request << std::endl;
    //     //     ii++;
    //     // }
    //     // double tn = ros::Time::now().toSec();
    //     // delta_t = tn - t0;
    //     // std::cout << std::fixed << std::setprecision(10) << tn - t0 << std::endl;
    //     // double t0 = ros::Time::now().toSec();
    //
    //     // ros::Timer timer_uav_state = n.createTimer( \
    //     //         ros::Duration(0.01),
    //     //         std::bind(&UavControl::get_uav_state, Uav)
    //     // );
    //     ros::Timer timer_uav_control = n.createTimer( \
    //             ros::Duration(0.01), std::bind(&UavControl::update, Uav)
    //     );
    //     ros::spin();
    //     // loop_rate.sleep();
    // // }
    // // gazebo::shutdown();
    return 0;
}
