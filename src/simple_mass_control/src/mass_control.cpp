#include "mass_control.hpp"


MassControl::MassControl(ros::NodeHandle *n)
{
    MassControl::update_parameters(n);
    MassControl::set_initial_pose(n);

    sub_state = n->subscribe("states", 1, MassControl::update_states);

    std::cout << "Properties:\n\tmass: " << m 
        << "\n\tgravity: " << g 
        << std::endl;
}


MassControl::~MassControl(void)
{
    ;
}


void MassControl::update_states(
    const simple_mass_control::states::ConstPtr &msg
)
{
    msg_to_vector(msg->x, x);
    msg_to_vector(msg->v, v);
}


void MassControl::print_msg(void)
{
    std::cout << "\n\nstates:"
        << "\n\tx: " << x 
        << "\n\tv: " << v
        << std::endl;
}


void MassControl::msg_to_vector(
    geometry_msgs::Vector3 msg_vec, ignition::math::Vector3d &vec
)
{
    vec[0] = msg_vec.x;
    vec[1] = msg_vec.y;
    vec[2] = msg_vec.z;
}


void MassControl::set_initial_pose(ros::NodeHandle *n)
{
    ros::ServiceClient client \
        = n->serviceClient<gazebo_msgs::SetModelState>( \
        "/gazebo/set_model_state"
    );

    geometry_msgs::Pose start_pose;
    start_pose.position.x = 0.0;
    start_pose.position.y = 0.0;
    start_pose.position.z = 0.0;
    start_pose.orientation.x = 0.0;
    start_pose.orientation.y = 0.0;
    start_pose.orientation.z = 0.0;
    start_pose.orientation.w = 0.0;

    gazebo_msgs::ModelState modelstate;
    gazebo_msgs::SetModelState setmodelstate;

    modelstate.model_name = (std::string) "unit_box";
    modelstate.reference_frame = (std::string) "world";
    modelstate.pose = start_pose;

    setmodelstate.request.model_state = modelstate;
    client.call(setmodelstate);

    ros::spinOnce();
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
