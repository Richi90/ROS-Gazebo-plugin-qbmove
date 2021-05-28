#include <sea_plugin/sea_plugin.h>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <boost/bind.hpp>
#include <math.h>
#include <string>
#include <ros/ros.h>
#include <sea_plugin/state_info.h>

using namespace gazebo;
using namespace std;

/*
  The purpose of this plugin function, started on each joint of your robot actuated with a SEA device, is to compute the
  elastic torque of the motor-spring unit and to pass it as torque that actuates the relative joint.
  The value of the stiffness is passed as external tag from the URDF plugin.
*/

void seaPlugin::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
{
  int argc = 0;
  char **argv;
  ros::init(argc, argv, "sea_Plugin");

  this->model = _parent;

  // Make sure the ROS node for Gazebo has already been initialized                                                                                    
  if (!ros::isInitialized())
  {
    ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
      << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
    return;
  }

  // Retrieve joint identifier and control mode from urdf tags
  this->joint_name =_sdf->GetElement("joint")->Get<string>();
  this->ns_name =_sdf->GetElement("namespace")->Get<string>(); // TODO: move out, passing it as ROS parameters
  this->K =_sdf->GetElement("stiffness")->Get<double>();
  this->flag_pub_el_tau =_sdf->GetElement("pub_eltau")->Get<bool>();
  this->flag_pub_state =_sdf->GetElement("pub_state")->Get<bool>();
  this->flag_sub_ext_tau =_sdf->GetElement("sub_ext_tau")->Get<bool>();

  // Everything-is-fine message
  std::string ok_msg = "SEA plugin on " + joint_name + " started!";
  ROS_WARN_STREAM(ok_msg);

  // Retrieve joint
  this->joint = this->model->GetJoint(joint_name);
  // Compose the name of the motor position publisher
  cmd_sub_name = ns_name + "/" + joint_name + "/theta_command";

  // Compose the name of the elastic torque publisher
  el_pub_name = ns_name + "/" + joint_name + "/tau_el_state";

  // Compose string name for the state publisher
  link_pub_name = ns_name + "/" + joint_name + "/link_state";

  // Compose string for the subscriber of external torque
  ext_tau_sub_name = ns_name + "/" + joint_name + "/ext_tau";

  // Subscribers and Publishers for the joint states and command
  sub = n.subscribe(cmd_sub_name, 10, &seaPlugin::getRef_callback, this);

  //pub = n.advertise<std_msgs::Float64>(cmd_pub_name, 500);

  if (this->flag_sub_ext_tau)
  {
    sub_ext = n.subscribe(ext_tau_sub_name, 10, &seaPlugin::getExtTau_callback, this);
  } 

  if (this->flag_pub_state)
  {
    pub_state = n.advertise<sea_plugin::state_info>(link_pub_name, 500);
  }
  if (this->flag_pub_el_tau)
  {
    pub_eltau = n.advertise<std_msgs::Float64>(el_pub_name, 500);
  }

  this->updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&seaPlugin::OnUpdate, this, _1));  

}

// Subscriber callbacks references
void seaPlugin::getRef_callback(const std_msgs::Float64& val)
{
    this->joint_ref = val;
}

// Subscriber callback for external torque
void seaPlugin::getExtTau_callback(const std_msgs::Float64& e_tau)
{
    this->ext_tau = e_tau;
}

// Main update function
void seaPlugin::OnUpdate(const common::UpdateInfo & /*_info*/)
{
  // Retrieve joint actual position
  this->q = this->joint->Position(0);
  this->dq = this->joint->GetVelocity(0);

  // Compute the elastic and link torques
  this->elastic_tau.data = this->K*(this->joint_ref.data - this->q);
  this->joint_tau.data = this->elastic_tau.data - this->Damp*this->dq;

  // Compute the elastic torque
  this->joint_tau.data = this->K*(this->joint_ref.data - this->q) - this->Damp*this->dq;

  // Set to the joint elastic torque
  this->joint->SetForce(0, this->joint_tau.data - this->ext_tau.data);

  // Publish elastic torque for driving motor
  if (this->flag_pub_el_tau)
  {
    pub_eltau.publish(this->elastic_tau);
  }

  // Publish whole joint state
  if (this->flag_pub_state)
  {
    this->joint_info.tau = this->joint_tau.data;
    this->joint_info.stiff = this->K;
    this->joint_info.q = this->q;
    this->joint_info.dq = this->dq;
    this->joint_info.ref_1 = this->joint_ref.data;
    this->joint_info.ref_2 = -1;
    pub_state.publish(this->joint_info);
  }

}