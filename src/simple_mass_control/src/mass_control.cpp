#include "mass_control.hpp"


MassControl::MassControl(ros::NodeHandle *n)
{
    MassControl::update_parameters(n);

    std::cout << "Properties:\n\tmass: " << m 
        << "\n\tgravity: " << g 
        << std::endl;
}


MassControl::~MassControl(void)
{
    ;
}


void MassControl::update_parameters(ros::NodeHandle *n)
{
    gazebo_msgs::GetPhysicsPropertiesRequest world_physics_request;
    gazebo_msgs::GetPhysicsPropertiesResponse world_physics; 
    ros::ServiceClient client_world_physics \
        = n->serviceClient<gazebo_msgs::GetPhysicsProperties>( \
        "/gazebo/get_physics_properties"
    );
    client_world_physics.call(world_physics_request, world_physics);
    const double g = -world_physics.gravity.z;

    gazebo_msgs::GetLinkPropertiesRequest mass_property_request;
    gazebo_msgs::GetLinkPropertiesResponse mass_properties;
    ros::ServiceClient client_mass_properties \
        = n->serviceClient<gazebo_msgs::GetLinkProperties>( \
        "/gazebo/get_link_properties"
    );      
    mass_property_request.link_name = (std::string) "unit_box::link";
    client_mass_properties.call(mass_property_request, mass_properties);
    double m = mass_properties.mass;
}
