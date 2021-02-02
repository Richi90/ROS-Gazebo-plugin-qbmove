#include <advanced_plugin/advanced_plugin.h>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <boost/bind.hpp>
#include <math.h>
#include <string>
#include <ros/ros.h>
#include <advanced_plugin/state_info.h>

using namespace gazebo;
using namespace std;

/*
  The purpose of this plugin function, started on each joint of your robot actuated with a qbMove device, is to compute 
  elastic torque of the two motor-spring units, and to sum them to create the torque that actuates the relative joint.
  This can be done by leveraging on three different control types:
  0) the plugin subscribes to two topics in which the motor references are passed /reference_1 and /reference_2
  1) the plugin subscribes to two topics in which the equilibirum position and preset values are passed /equilibirum_pos and /preset
  2) the plugin uses only one topic, named /torque, (the other is not used) to drive directly the torque of the joint.
  In order to exchange information to the other similar topic (on the motors), this topic publishes also the elastic torques computed.
*/

void advancedPlugin::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
{
  int argc = 0;
  char **argv;
  ros::init(argc, argv, "qbmoveAdvanced_plugin");

  this->model = _parent;

  // Make sure the ROS node for Gazebo has already been initialized                                                                                    
  if (!ros::isInitialized())
  {
    ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
      << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
    return;
  }

  // Retrieve namespace and control mode from a configuration file
  ros::param::get("namespace", this->ns_name);
  ros::param::get("T_sample", this->T_sample);

  // Set node's rate
  // ros::Rate fs(1/T_sample); // TO TEST IT!

  // Retrieve joint identifier and control mode from urdf tags
  this->joint_name =_sdf->GetElement("joint")->Get<string>();
  this->cont_type =_sdf->GetElement("control_type")->Get<int>();
  this->flag_pub_el_tau =_sdf->GetElement("pub_eltau")->Get<bool>();
  this->flag_pub_state =_sdf->GetElement("pub_state")->Get<bool>();
  this->flag_sub_ext_tau =_sdf->GetElement("sub_ext_tau")->Get<bool>();

  // Everything-is-fine message
  std::string ok_msg = "qbmove Advanced plugin on " + joint_name + " started with controller type [" + std::to_string(cont_type) + "]!";
  ROS_WARN_STREAM(ok_msg);

  // Retrieve joint
  this->joint = this->model->GetJoint(joint_name);

  // Initialize elastic motor-spring unit torques
  this->elastic_tau1.data = 0;
  this->elastic_tau2.data = 0;
  this->joint_tau.data = 0;
  this->ref_1.data = 0;
  this->ref_2.data = 0;

  // Compose string name for the publishers removing last words from joint name (SUPPOSING THAT QB NODE CONVENTION IS USED) 
  //    and add the suffix according to the config file!! 
  cmd_pub1_name = ns_name + "/" + joint_name + "/tau_el1_state";
  cmd_pub2_name = ns_name + "/" + joint_name + "/tau_el2_state";

  // Compose string name for the state publisher
  link_pub_name = ns_name + "/" + joint_name + "/state";

  // Compose string name for the subscribers removing last words from joint name (SUPPOSING THAT QB NODE CONVENTION IS USED) 
  //    and add the suffix according to the config file!! 
  // ----->  TODO: to much customized, make it more general!  <------------
  // cmd_ref1_name = ns_name + "/" + joint_name.erase(joint_name.length()-11, 11) + "/reference_1";
  // cmd_ref2_name = ns_name + "/" + joint_name.erase(joint_name.length()-11, 11) + "/reference_2";
  if (this->cont_type==0)       // references used as motor positions (**leg**_ref_1 and **leg**_ref_2, respectively)
  {
    cmd_ref1_name = ns_name + "/" + joint_name + "/reference_1";
    cmd_ref2_name = ns_name + "/" + joint_name + "/reference_2";
  }
  if (this->cont_type==1)       // references used as equilibrium postion (ref_1) and preset values (ref_2)
  {
    cmd_ref1_name = ns_name + "/" + joint_name + "/equilibrium_pos";
    cmd_ref2_name = ns_name + "/" + joint_name + "/preset";
  }
  if (this->cont_type==2)       // references (ref_1) used as direct torques of the joints
  {
    cmd_ref1_name = ns_name + "/" + joint_name + "/torque";
    cmd_ref2_name = ns_name + "/" + joint_name + "/disabled";
  }
  if (this->cont_type==3)       // references retrieved from the (torque-controlled) motor dynamics of the other plugins
  {
    int len_name = joint_name.length();
    cmd_ref1_name = ns_name + "/" + joint_name.erase(len_name-11, 11) + "motor_1_joint/reference_1";
    cmd_ref2_name = ns_name + "/" + joint_name.erase(len_name-11, 11) + "motor_2_joint/reference_2";
  }

  // Compose string for the subscriber of external torque
  ext_tau_sub_name = ns_name + "/" + joint_name + "/ext_tau";

  // Subscribers and Publishers for the joint states and command
  sub_1 = n.subscribe(cmd_ref1_name, 10, &advancedPlugin::getRef1_callback, this);
  sub_2 = n.subscribe(cmd_ref2_name, 10, &advancedPlugin::getRef2_callback, this);
  if (this->flag_sub_ext_tau)
  {
    sub_ext = n.subscribe(ext_tau_sub_name, 10, &advancedPlugin::getExtTau_callback, this);
  } 
  if (this->flag_pub_el_tau)
  {
    pub_1 = n.advertise<std_msgs::Float64>(cmd_pub1_name, 500);
    pub_2 = n.advertise<std_msgs::Float64>(cmd_pub2_name, 500);
  }
  if (this->flag_pub_state)
  {
    pub_state = n.advertise<advanced_plugin::state_info>(link_pub_name, 500);
  }

  this->updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&advancedPlugin::OnUpdate, this, _1));  

}

// Saturation for elastic deflections
double advancedPlugin::saturate(double val, double max_val)
{
    if (val > max_val) {  
        val = max_val;
    }
    if (val < -max_val) {
        val = -max_val;
    }
    return val;
}

// Elastic function torque model for qbMove Advanced (datasheet)
double advancedPlugin::elastic_fun(double link, double theta)
{
    double tau_el, defl;
    // Elastic model parameters
    double a = 6.7328;
    double k = 0.0222;

    defl = saturate(link - theta, 0.8);
    return tau_el = -k*sinh(a*defl);
}

// Elastic function stiffness model for qbMove Advanced (datasheet)
double advancedPlugin::stiffness_fun(double link, double theta)
{
    double sig_el, defl;
    // Elastic model parameters
    double a = 6.7328;
    double k = 0.0222;

    defl = saturate(link - theta, 0.8);
    return sig_el = a*k*cosh(a*defl);
}

// Subscriber callbacks references
void advancedPlugin::getRef1_callback(const std_msgs::Float64& val_1)
{
    this->ref_1 = val_1;
}
void advancedPlugin::getRef2_callback(const std_msgs::Float64& val_2)
{
    this->ref_2 = val_2;
}

// Subscriber callback for external torque
void advancedPlugin::getExtTau_callback(const std_msgs::Float64& e_tau)
{
    this->ext_tau = e_tau;
}

// Switch among controllers
double advancedPlugin::SwitchControl(int cont_sel)
{
  if (cont_sel==0 || cont_sel==3)         // references used as motor positions (**leg**_ref_1 and **leg**_ref_2, respectively)
  {
      // Elstic torque computation with additional damping added (required for a better simulation)
      this->elastic_tau1.data = advancedPlugin::elastic_fun(this->q, this->ref_1.data);
      this->elastic_tau2.data = advancedPlugin::elastic_fun(this->q, this->ref_2.data);
      this->stiffness_1.data = advancedPlugin::stiffness_fun(this->q, this->ref_1.data);
      this->stiffness_2.data = advancedPlugin::stiffness_fun(this->q, this->ref_2.data);
      return this->elastic_tau1.data + this->elastic_tau2.data - Damp_link*this->dq; // joint torque
      // std::cout << "Control mode: motor positions! ----- ";

  }
  if (cont_sel==1)                        // references used as equilibrium postion (ref_1) and preset values (ref_2)
  {
      double th1_temp, th2_temp; // used to store temporarly the motor references as:
                                 //          th1_temp = eqpos(ref_1) + preset(ref_2)
                                 //          th2_temp = eqpos(ref_1) - preset(ref_2)
      
      th1_temp = this->ref_1.data + this->ref_2.data;
      th2_temp = this->ref_1.data - this->ref_2.data;
      this->elastic_tau1.data = advancedPlugin::elastic_fun(this->q, th1_temp);
      this->elastic_tau2.data = advancedPlugin::elastic_fun(this->q, th2_temp);
      this->stiffness_1.data = advancedPlugin::stiffness_fun(this->q, th1_temp);
      this->stiffness_2.data = advancedPlugin::stiffness_fun(this->q, th2_temp);
      return this->elastic_tau1.data + this->elastic_tau2.data - Damp_link*this->dq; // joint torque
      //std::cout << "Control mode: eq.pos/preset! ----- ";

  }
  if (cont_sel==2)                        // references (ref_1) used as direct torques of the joints
  {
      // Reference (only ref_1) copied as pure joint torques. The additional damping is added for a better simulation!
      return this->ref_1.data - Damp_link*dq;
      //std::cout << "Control mode: joint torques! ----- ";

  }
}

// Main update function
void advancedPlugin::OnUpdate(const common::UpdateInfo & /*_info*/)
{

  // Retrieve joint position and velocity
  this->q = this->joint->GetPosition(0);
  this->dq = this->joint->GetVelocity(0);

  // Compose the elstic torque and stiffness according to the control mode selected
  this->joint_tau.data = advancedPlugin::SwitchControl(this->cont_type);
  this->joint_stiff.data = this->stiffness_1.data + this->stiffness_2.data;

  // Set torque to the joint
  this->joint->SetForce(0, this->joint_tau.data - this->ext_tau.data);

  // Publish elastic torque for driving motors
  if (this->flag_pub_el_tau)
  {
    pub_1.publish(this->elastic_tau1);
    pub_2.publish(this->elastic_tau2);
  }

  // Publish whole joint state
  if (this->flag_pub_state)
  {
    this->joint_info.tau = this->joint_tau.data;
    this->joint_info.stiff = this->joint_stiff.data;
    this->joint_info.q = this->q;
    this->joint_info.dq = this->dq;
    this->joint_info.ref_1 = this->ref_1.data;
    this->joint_info.ref_2 = this->ref_2.data;
    pub_state.publish(this->joint_info);
  }
}