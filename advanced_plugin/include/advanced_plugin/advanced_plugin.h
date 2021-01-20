#include <gazebo/common/Plugin.hh>
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <advanced_plugin/state_info.h>

namespace gazebo{
  
  class advancedPlugin : public ModelPlugin {
  
  public:
    // Constructor
    advancedPlugin() : ModelPlugin(){}

    void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);

    void OnUpdate(const common::UpdateInfo & /*_info*/);

    void printMe();

    double saturate(double val, double max_val);

    double elastic_fun(double link, double theta);

    double stiffness_fun(double link, double theta);

    double SwitchControl(int cont_sel);

    void getRef1_callback(const std_msgs::Float64& val_1);

    void getRef2_callback(const std_msgs::Float64& val_2);

    void getExtTau_callback(const std_msgs::Float64& tau);

  private:

    // Command name to create a specific publisher and subscribers
    std::string cmd_pub1_name;
    std::string cmd_pub2_name;
    std::string cmd_ref1_name;
    std::string cmd_ref2_name;

    // String names of the topic from which retrieve the desired motor variables
    std::string mot1_sub_name;
    std::string mot2_sub_name;
    std::string link_pub_name;

    // String name for the external torque
    std::string ext_tau_sub_name;

    // Joint name and namespace retrieved from the urdf
    std::string joint_name;
    std::string ns_name = "toBEassigned";

    // Enable publish and subscribe to specific topics (default "false")
    bool flag_pub_el_tau = "false";
    bool flag_pub_state = "false";
    bool flag_sub_ext_tau = "false";

    // Control type retrieved from the urdf
    int cont_type = 0;

    // Command message
    std_msgs::Float64 joint_tau, ext_tau, joint_stiff;

    // Reference messages
    std_msgs::Float64 ref_1, ref_2;

    // Link position messages
    std_msgs::Float64 link_q;

    // Pointer to the model 
    physics::ModelPtr model;

    // Pointer to output shaft joint 
    physics::JointPtr joint;

    // Pointer to the update event connection 
    event::ConnectionPtr updateConnection;

    // Ros variables for the node
    ros::Publisher pub_1;   // for motor torque 1
    ros::Publisher pub_2;   // for motor torque 2
    ros::Publisher pub_state;// for link position
    ros::Subscriber sub_1;  // for references 1
    ros::Subscriber sub_2;  // for references 2
    ros::Subscriber sub_ext;// for eternal torque
    ros::NodeHandle n;

    // Link variables
    double q, dq;

    // Motor-spring unit torques
    std_msgs::Float64 elastic_tau1, elastic_tau2;
    std_msgs::Float64 stiffness_1, stiffness_2;

    // Other variables
    double Damp_link = 0.05;

    // Sample time
    double T_sample;

    // Custom message for joint info
    advanced_plugin::state_info joint_info;

  };

  GZ_REGISTER_MODEL_PLUGIN(advancedPlugin)
}