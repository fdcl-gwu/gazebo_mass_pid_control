#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>

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
        this->update_connection = event::Events::ConnectWorldUpdateBegin(
            std::bind(&MassPlugin::update, this));
        this->pub_state = n.advertise<simple_mass_control::states>("states", 10);
    }

    void update(void)
    {
        this->model->SetLinearVel(ignition::math::Vector3d(0.3, 0, 0.3));
        // this->link->SetForce(ignition::math::Vector3d(0.5, 0.0, 9.9));
        
        state.header.stamp = ros::Time::now();
        state.header.frame_id = (std::string) "mass_states";
        
        this->pub_state.publish(state);
    }

private:
    physics::ModelPtr model;
    physics::LinkPtr link;
    
    event::ConnectionPtr update_connection;
    ros::Publisher pub_state; 
    ros::NodeHandle n;
    simple_mass_control::states state;
};

// Register this plugin with the simulator
 GZ_REGISTER_MODEL_PLUGIN(MassPlugin)

}  // end of namespace gazebo

