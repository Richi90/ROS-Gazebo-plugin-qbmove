#include <gazebo/common/Plugin.hh>
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <sea_plugin/state_info.h>

namespace gazebo{
  
  class seaPlugin : public ModelPlugin {
  
  public:
    // Constructor
    seaPlugin() : ModelPlugin(){}

    void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);

    void OnUpdate(const common::UpdateInfo & /*_info*/);

    void getRef_callback(const std_msgs::Float64& val);

    void getExtTau_callback(const std_msgs::Float64& tau);

  private:

    // Command name to create a specific publisher and subscribers
    std::string cmd_sub_name;
    std::string cmd_pub_name;
    std::string link_pub_name;
    std::string el_pub_name;

    // Joint name and namespace retrieved from the urdf
    std::string joint_name;
    std::string ns_name;

    // String name for the external torque
    std::string ext_tau_sub_name;

    // Pointer to the model 
    physics::ModelPtr model;

    // Pointer to output shaft joint 
    physics::JointPtr joint;

    // Pointer to the update event connection 
    event::ConnectionPtr updateConnection;    

    // Define publisher and subscriber
    ros::Publisher pub;     // for link position
    ros::Subscriber sub;    // for joint position reference
    ros::Publisher pub_state;// for link position
    ros::Publisher pub_eltau;// for link position
    ros::Subscriber sub_ext;// for eternal torque
    ros::NodeHandle n;
    
    // Reference messages
    std_msgs::Float64 joint_ref;

    // Elastic torque 
    std_msgs::Float64 elastic_tau;
 
    // Temporarly messages to publish
    std_msgs::Float64 joint_tau, ext_tau;

    // Define the joint variables
    double K;         // [Nm/rad]
    double Damp = 0.05;    // [Nms/rad] 
    double q, dq;

    // Enable publish and subscribe to specific topics (default "false")
    bool flag_pub_state = "false";
    bool flag_sub_ext_tau = "false";
    bool flag_pub_el_tau = "false";
    
    // Custom message for joint info
    sea_plugin::state_info joint_info;

  };

  GZ_REGISTER_MODEL_PLUGIN(seaPlugin)
}