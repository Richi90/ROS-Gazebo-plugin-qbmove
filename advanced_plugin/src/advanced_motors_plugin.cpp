#include <advanced_plugin/advanced_motors_plugin.h>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <boost/bind.hpp>
#include <math.h>
#include <string>
#include <ros/ros.h>

using namespace gazebo;
using namespace std;

/*
  The purpose of this plugin function, started on each of the qbmove motors, is to retrieve the elastic torques ( from the other 
  plugin that is started on the shaft link ) and to subscribe to motor torques passed by the user as commands. 
  Then, it evaluates the motors dynamics, using the two "fake" motor joints inserted on the URDF, and it publishes the motor states,
  i.e., the motor angles, back to the other plugin as a references. (See the other plugin description for other details.)
*/

void advancedMotorsPlugin::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
{
  int argc = 0;
  char **argv;
  ros::init(argc, argv, "qbmoveMotorsAdvanced_plugin");

  this->model = _parent;

  // Make sure the ROS node for Gazebo has already been initialized                                                                                    
  if (!ros::isInitialized())
  {
    ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
      << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
    return;
  }

  // Retrieve namespace from a configuration file
  ros::param::get("namespace", this->ns_name);
  ros::param::get("T_sample", this->T_sample);

  // Set node's rate
  // ros::Rate fs(1/T_sample); // TO TEST IT!

  // Retrieve joint identifier and control mode from urdf tags
  joint_name =_sdf->GetElement("joint")->Get<string>();
  motor_num =_sdf->GetElement("motor")->Get<int>();

  // Initialize torque command to zero
  this->mot_cmd_tau.data = 0;

  // Everything-is-fine message
  std::string ok_msg = "qbmove motors Advanced plugin on " + joint_name + " started!";
  ROS_WARN_STREAM(ok_msg);

  // Retrieve joint
  this->joint = this->model->GetJoint(joint_name);

  // Create the string name for the publisher according to the number of motor retrieved from the urdf
  this->cmd_ref_name = ns_name + "/" + joint_name + "/reference_" + std::to_string(motor_num);

  // Create the string name for the publisher according to the number of motor retrieved from the urdf
  this->mottau_sub_name = ns_name + "/" + joint_name + "/command";

  // Create the string name for the subscriber according to the number of motor retrieved from the urdf 
  //    and according to the name of the published variable from the qbmove advanced plugin on the shaft.
  //    This implies to replace the *motor_XX_joint* name with *shaft_joint* as follows. 
  int cut_link_name = joint_name.length();
  std::string cut_sub_name = joint_name.erase(cut_link_name-13, 13) + "shaft_joint";
  this->eltau_sub_name = ns_name + "/" + cut_sub_name + "/tau_el" + std::to_string(motor_num) + "_state";

  // Create the string for the subscriber to the link position
  this->link_sub_name = ns_name + "/" + cut_sub_name + "/state";

  // Subscribers and Publishers for the joint states and command 
  sub_el = n.subscribe(eltau_sub_name, 10, &advancedMotorsPlugin::getElTau_callback, this); 
  sub_mot = n.subscribe(mottau_sub_name, 10, &advancedMotorsPlugin::getMotTau_callback, this); 
  sub_q = n.subscribe(link_sub_name, 10, &advancedMotorsPlugin::getLink_callback, this);
  pub = n.advertise<std_msgs::Float64>(cmd_ref_name, 500);

  this->updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&advancedMotorsPlugin::OnUpdate, this, _1));  
}

// Subscriber callback to elastic function (from other plugin)
void advancedMotorsPlugin::getElTau_callback(const std_msgs::Float64& val)
{
    this->el_tau = val;
}

// Subscriber callback to motor function (from user command)
void advancedMotorsPlugin::getMotTau_callback(const std_msgs::Float64& val)
{
    this->mot_cmd_tau = val;
}

// Subscriber callback to link position (from other plugin)
void advancedMotorsPlugin::getLink_callback(const std_msgs::Float64& val)
{
    this->link_q = val;
}

// Saturation for elastic deflections
double advancedMotorsPlugin::saturate(double val, double max_val)
{
    if (val > max_val) {  
        val = max_val;
    }
    if (val < -max_val) {
        val = -max_val;
    }
    return val;
}

// Elastic function model for qbMove Advanced (datasheet)
double advancedMotorsPlugin::elastic_fun(double link, double theta)
{
    double tau_el, defl;
    // Elastic model parameters
    double a = 6.7328;
    double k = 0.0222;

    defl = saturate(link - theta, 0.8);
    return tau_el = -k*sinh(a*defl);
}

// Main update function
void advancedMotorsPlugin::OnUpdate(const common::UpdateInfo & /*_info*/)
{
  // Retrieve motor position and velocity
  this->th.data = this->joint->GetPosition(0); 
  this->th_red.data = this->th.data*(pow(gear_ratio,2));
  this->dth.data = this->joint->GetVelocity(0);
  
  this->tau_elastic_temp = advancedMotorsPlugin::elastic_fun(this->link_q.data, this->th.data);

  // Compute motor torque from commanded, elastic and damping (motor dynamics emulated!)
  this->mot_tau = this->mot_cmd_tau.data - this->Damp_mot*this->dth.data; // /gear_ratio

  // Set motor torque to the joint
  this->joint->SetForce(0, this->mot_tau);

  // Publish motor reference to the other plugin for computing elastic torque and drive the VSA link   
  pub.publish(this->th);

  std::cout << tau_elastic_temp << "\t" << this->el_tau.data << "\t" << this->link_q.data  << std::endl;
}