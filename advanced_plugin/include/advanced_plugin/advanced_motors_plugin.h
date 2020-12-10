#include <gazebo/common/Plugin.hh>
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <ros/ros.h>
#include <std_msgs/Float64.h>

namespace gazebo{
  
  class advancedMotorsPlugin : public ModelPlugin {
  
  public:
    // Constructor
    advancedMotorsPlugin() : ModelPlugin(){}

    void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);

    void OnUpdate(const common::UpdateInfo & /*_info*/);

    double saturate(double val, double max_val);

    double elastic_fun(double link, double theta);

    void getElTau_callback(const std_msgs::Float64& val);

    void getMotTau_callback(const std_msgs::Float64& val);

    void getLink_callback(const std_msgs::Float64& val);

  private:

    // Joint name and namespace retrieved from the urdf
    std::string joint_name;
    std::string ns_name = "toBEassigned";
    int motor_num;

    // String name for the publisher and for the subscribers
    std::string cmd_ref_name;
    std::string eltau_sub_name;
    std::string mottau_sub_name;
    std::string link_sub_name;

    // Messages to subscribe on
    std_msgs::Float64 mot_cmd_tau; // commanded from user
    std_msgs::Float64 el_tau;
    std_msgs::Float64 th, dth;     // motor variables and reference to pass to the other plugin

    // Link position
    std_msgs::Float64 link_q;

    // Pointer to the model 
    physics::ModelPtr model;

    // Pointer to output shaft joint 
    physics::JointPtr joint;

    // Pointer to the update event connection 
    event::ConnectionPtr updateConnection;

    // Ros variables for the node
    ros::Publisher pub;       // for motor reference (angle)
    ros::Subscriber sub_el;   // for elastic torque
    ros::Subscriber sub_mot;  // for motor torque
    ros::Subscriber sub_q;    // for link position
    ros::NodeHandle n;

    // Motor variables and parameters
    double mot_tau;
    double Damp_mot = 0.00000742;
    int gear_ratio = 205;

    // Sample time
    double T_sample;

  };

  GZ_REGISTER_MODEL_PLUGIN(advancedMotorsPlugin)
}