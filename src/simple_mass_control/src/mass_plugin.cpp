#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>

#include <ignition/math.hh>
#include <ignition/math/Vector3.hh>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Wrench.h>

#include <ros/ros.h>
#include "std_msgs/String.h"

#include "simple_mass_control/states.h"

namespace gazebo
{

class MassPlugin : public ModelPlugin
{

public: 
    void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
    {
        this->model = _parent;
        this->link = this->model->GetLink((std::string) "link");

        // Listen to the update event. This event is broadcast every
        // simulation iteration.
        this->update_connection = event::Events::ConnectWorldUpdateBegin( \
            std::bind(&MassPlugin::update, this));
        this->pub_state = n.advertise<simple_mass_control::states>( \
            "states", 10);
    }


    void update(void)
    {
        // this->model->SetLinearVel(ignition::math::Vector3d(0.3, 0, 0.3));
        
        state.header.stamp = ros::Time::now();
        state.header.frame_id = (std::string) "mass_states";

        ignition::math::Pose3d pose = this->model->WorldPose();
        vector_to_msg(pose.Pos(), state.x);
        
        ignition::math::Quaterniond rot = pose.Rot();
        state.R.x = rot.Yaw();
        state.R.y = rot.Pitch();
        state.R.z = rot.Roll();

        vector_to_msg(this->model->WorldLinearVel(), state.v);
        vector_to_msg(this->model->WorldAngularVel(), state.W);
        vector_to_msg(this->model->WorldLinearAccel(), state.a);

        this->pub_state.publish(state);

        ignition::math::Vector3d xd(5.0, 0.0, 0.5);
        ignition::math::Vector3d vd(0.0, 0.0, 0.0);
        
        ex = pose.Pos() - xd;
        ev = this->model->WorldLinearVel() - vd;
        ei = ei + ex * 0.001;

        force = - 10.0 * ex - 8.0 * ev - 5.0 * ei;
        double m = 1.0;
        double g = 9.8;
        force[2] = force[2] + m * g;

        this->link->SetForce(force);

        std::cout << ex << std::endl;
    }


    void vector_to_msg(
            ignition::math::Vector3d vec, geometry_msgs::Vector3 &msg_vec
    )
    {
       msg_vec.x = vec[0];
       msg_vec.y = vec[1];
       msg_vec.z = vec[2];
    } 


private:
    physics::ModelPtr model;
    physics::LinkPtr link;
    
    event::ConnectionPtr update_connection;
    ros::Publisher pub_state; 
    ros::NodeHandle n;
    simple_mass_control::states state;

    ignition::math::Vector3d ex;
    ignition::math::Vector3d ev;
    ignition::math::Vector3d ei;
    ignition::math::Vector3d force;
};


// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(MassPlugin)


}  // end of namespace gazebo

