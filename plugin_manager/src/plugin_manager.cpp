#include <ros/ros.h>
#include <ros/rate.h>
#include <std_msgs/Float64.h>
#include <math.h>
#include <string>
#include <boost/scoped_ptr.hpp>
#include <eigen3/Eigen/Eigen>
#include <plugin_manager/QbAdvanceRefEq.h>
#include <std_msgs/Float64MultiArray.h> 
#include <sensor_msgs/JointState.h>
#include <advanced_plugin/state_info.h>

# define PI 3.14159

int joints_n;
std::string ns_name;

std::vector<ros::Publisher> pub_eq;
std::vector<ros::Publisher> pub_preset;
ros::Publisher pub_robot_state;

std::vector<ros::Subscriber> sub_joints_state;
ros::Subscriber sub_Eq_Des, sub_ref1, sub_ref2;

std::vector<double> preset;
std::vector<double> qDes, presetDes;
std::vector<double> q, dq, tau, stiff;

std_msgs::Float64 msg_eq, msg_preset;
std_msgs::Float64MultiArray ref_1, ref_2;

std::vector<std::string> joint_name;
std::vector<std::string> topics_eq, topics_preset, topics_state;
std::string topic_state_name;
std::string sub_topic_name;

sensor_msgs::JointState robot_state;

double temp_val;

void DesEqPositionCallback(const plugin_manager::QbAdvanceRefEqConstPtr &msg)
{
  qDes = msg->EqDes;
  presetDes = msg->Preset;
}

void JointStateCallback(const advanced_plugin::state_infoConstPtr &msg, int i)
{
  q[i] = msg->q;
  dq[i] = msg->dq;
  tau[i] = msg->tau;
  stiff[i] = msg->stiff;
}

void Ref1_Callback(const std_msgs::Float64MultiArray &msg)
{
  ref_1 = msg;
}

void Ref2_Callback(const std_msgs::Float64MultiArray &msg)
{
  ref_2 = msg;
}

void Initialization(ros::NodeHandle n_)
{
  // Check if namespace is passed and get it
  if (!n_.getParam("namespace", ns_name))
  {
    ROS_ERROR("Specify namespace of robot");
    exit(1);
  }
  // Check if joint number is passed and get it
  if (!n_.getParam("joints_number", joints_n))
  {
    ROS_ERROR("Specify number of joint");
    exit(1);
  }

  // Resize array variables
  qDes.resize(joints_n);
  presetDes.resize(joints_n);
  //qDes.assign(joints_n,0.0);
  preset.resize(joints_n);
  pub_eq.resize(joints_n);
  pub_preset.resize(joints_n);
  sub_joints_state.resize(joints_n);
  joint_name.resize(joints_n);
  topics_eq.resize(joints_n);
  topics_preset.resize(joints_n);
  topics_state.resize(joints_n);
  ref_1.data.resize(joints_n);
  ref_2.data.resize(joints_n);
  q.resize(joints_n);
  dq.resize(joints_n);
  tau.resize(joints_n);
  stiff.resize(joints_n);
  robot_state.name.resize(joints_n);
  robot_state.position.resize(joints_n);
  robot_state.velocity.resize(joints_n);
  robot_state.effort.resize(joints_n);

  // Check if preset vector is passed  and get it 
  // TODO: move this to another subscriber topic!!
  if (!n_.getParam("initial_preset", preset))
  {
    ROS_ERROR("Specify the initial preset vector");
    exit(1);
  }
  // Check if joint name vector is passed and get it
  if (!n_.getParam("joints_name", joint_name))
  {
    ROS_ERROR("Specify the joint name vector");
    exit(1);
  }
  
  // Create the publisher topics' names
  for(int i = 0; i<joints_n; i++)
  {
    topics_eq[i] = "/"+  ns_name + "/" + joint_name[i] + "/equilibrium_pos";
    topics_preset[i] = "/"+  ns_name + "/" + joint_name[i] + "/preset";
    topics_state[i] = "/"+  ns_name + "/" + joint_name[i] + "/state";
  }
  topic_state_name = "/"+  ns_name + "/robot_state";

  // Advertise topics' publishers
  for(int i = 0; i<joints_n; i++)
  {
    pub_eq[i] =  n_.advertise<std_msgs::Float64>(topics_eq[i], 10);
    pub_preset[i] =  n_.advertise<std_msgs::Float64>(topics_preset[i], 10);
  }
  // Advertise robot state publisher
  pub_robot_state = n_.advertise<sensor_msgs::JointState>(topic_state_name, 10);

  // Subscribe to equilibirum position topics
  if (!n_.getParam("sub_topic_name", sub_topic_name))
  {
    ROS_ERROR("Specify the name of the subscriber topic");
    exit(1);
  }
  sub_Eq_Des = n_.subscribe(sub_topic_name, 10, &DesEqPositionCallback);

  // Subscribe to the joint state
  for(int i = 0; i<joints_n; i++)
  {
    sub_joints_state[i] = n_.subscribe<advanced_plugin::state_info>(topics_state[i], 10, boost::bind(JointStateCallback, _1, i));
  }

  std::string sub_ref1_name = ns_name + "/reference_1";
  std::string sub_ref2_name = ns_name + "/reference_2";

  sub_ref1 = n_.subscribe(sub_ref1_name, 10, &Ref1_Callback);
  sub_ref2 = n_.subscribe(sub_ref2_name, 10, &Ref2_Callback);

}

//-----------------------------------------------------
//                                                 main
//-----------------------------------------------------
int main(int argc, char **argv)
{
  double rateHZ = 100;
  
  ros::init(argc, argv, "plugin_manager_node");
  
  ros::NodeHandle n_;
  
  ros::Rate r(rateHZ);
  
  Initialization(n_);
 
  while(ros::ok())
  {

    for(int i = 0; i<joints_n; i++)
    {
      msg_eq.data = ref_1.data[i];
      msg_preset.data = ref_2.data[i];

      pub_eq[i].publish(msg_eq);
      pub_preset[i].publish(msg_preset);

      robot_state.name[i] = joint_name[i];
      robot_state.position[i] = q[i];
      robot_state.velocity[i] = dq[i];
      robot_state.effort[i] = tau[i];
    }

    pub_robot_state.publish(robot_state);

    ros::spinOnce();
    r.sleep();
	  
  }

  return 0;
}