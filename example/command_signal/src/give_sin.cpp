// ROS includes
#include "ros/ros.h"
#include "ros/time.h"
#include <math.h>
#include <string>
#include <std_msgs/Float64MultiArray.h>

using namespace std;

// Variables
ros::Publisher pub_cmd1;
ros::Publisher pub_cmd2;
std_msgs::Float64MultiArray cmd_1, cmd_2;
ros::Time init_time, curr_time, prev_time;

// Main fuction
int main(int argc, char **argv) {
    ros::init(argc, argv, "giveSinusoid");
    ros::NodeHandle n;
    int n_joints;
    double T_sample;
    double start_time, stop_time, t_fin, sin_amplitude, sin_omega;
    double preset;
    std::string ns_name;

    // Get number of cubes and stiffness from external ROS .confg file and resize variables
    n.getParam("n_joints", n_joints);
    n.getParam("T_sample", T_sample);
    n.getParam("start_time", start_time);
    n.getParam("stop_time", stop_time);
    n.getParam("t_fin", t_fin);
    n.getParam("sin_amplitude", sin_amplitude);
    n.getParam("sin_omega", sin_omega);
    n.getParam("preset", preset);
    n.getParam("namespace", ns_name);

    ros::Rate fs(1 / T_sample);

    // Initialize or resize variables
    cmd_1.data.resize(n_joints);
    cmd_2.data.resize(n_joints);

    // Initialize command values to zero
    for (int j = 0; j < n_joints; j++)
    {
        cmd_1.data[j] = 0;
        cmd_2.data[j] = 0;
    }

    // Publish the motors' state
    std::string pub1_name = "/" + ns_name + "/reference_1";
    std::string pub2_name = "/" + ns_name + "/reference_2";

    pub_cmd1 = n.advertise<std_msgs::Float64MultiArray>(pub1_name, 500);
    pub_cmd2 = n.advertise<std_msgs::Float64MultiArray>(pub2_name, 500);

    init_time = ros::Time::now();

    while (ros::ok())
    {
        curr_time = ros::Time::now();
        double act_time = curr_time.toSec() - init_time.toSec();

        // compute cycle time
        double delta_time = ( curr_time.toSec() - prev_time.toSec() );

        // Call the service to set command to each actuator
        std::cout << "\r[TIMES] [" + std::to_string(act_time) + "] [dT = " + std::to_string(delta_time) + "]\n";
        std::cout << std::flush;

        for (int j = 0; j < n_joints; j++)
        {
            // Rise signal
            if(act_time >= 0)
            {
                std::cout << "\rSignal started!";
                cmd_1.data[j] = sin_amplitude*sin(sin_omega*act_time);
                cmd_2.data[j] = sin_amplitude*sin(sin_omega*act_time);
            }
            // Kill node
            if(act_time >= t_fin)
            {
                ros::shutdown();
            }
        }

        std::cout << std::flush;

        // Publish motor states
        pub_cmd1.publish(cmd_1);
        pub_cmd2.publish(cmd_2);

        // Update old values
        prev_time = curr_time;

        fs.sleep();
        ros::spinOnce();
    }

/*    ros::waitForShutdown();
*/
    return 0;
}
