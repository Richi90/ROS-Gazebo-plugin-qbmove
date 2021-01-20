# ROS-Gazebo-plugin-qbmove
ROS-Gazebo plugin for simulate the elastic bheavior of the qbmove actuators in Gazebo. 
For those not familiar with these actuators, in brief, the *qbMove* is a compliant actuator with varible stiffness, based on the agonistic-antagonistic principle presented in [Catalano M. et. al.](https://ieeexplore.ieee.org/stamp/stamp.jsp?arnumber=5980457), developed by [qbRobotics](https://qbrobotics.com/).

The plugin supposes to be able to control the motor position references of the real qbmoves. 
_I'm planning to include the motor dynamics as well, from which test torque controllers, developping another complementary plugin._

## Usage 
Clone the packages, one implementing the plugin and the others implementing a template example and the qbRobotics utils, in your current catkin workspace.

There are two ways of using the **ROS-Gazebo-plugin-qbmove** properly:
- Standalone, within a simple URDF;
- Embedded, leveraging on the ROS-qbmove nodes inside the utils folder.

For the first solution, the advanced plugin must be inserted directly into the URDF, associated to each joints of your robot (see below).

For the second solution the advanced plugin is embedded into the **ROS-qbmove** (or **qbmove-ros**) package, thus it is required. 
In order to include the plugin, you have to replace the _/qbmove_description_ folder inside the **qbmove-ros** package with the one inside the _/qbmove_utils_ folder. (**NOTE:** that package has been modified w.r.t. the qbRobotics!)

# URDF template 
You can find a simple usage template inside the _/example/template_description_ folder. 

## Robot namespace definition
The namespace of your robot (required) is defined separately in a **yaml** file, inside the _/config_ folder as follows
   ```yaml
   ## Namespace of the robot
   namespace: "your_namespace"      # default = "toBEassigend"
   ```
and is loaded inside the **launch** file with the following
   ```
  	<!-- Load the parameter for the compliant plugin from the configuration file -->
  	<rosparam command="load" file="$(find template_description)/config/advanced_config.yaml"/>
   ```

## Plugin insertion 
The plugin can be inserted to each revolute (or continous) joint (_your_joint_) of your robot as follows
   ```xml
    <gazebo>
        <plugin name="advanced_vsa_j1" filename="libadvanced_plugin.so">
            <joint>your_joint_name</joint>
            <control_type>your_control_mode</control_type>
            <pub_eltau>true</pub_eltau>
            <pub_state>true</pub_state>
            <sub_ext_tau>true</sub_ext_tau>
        </plugin>
    </gazebo>
  ```
## Available topics
According to the control mode tag seleceted, different topics are generated to allow publishing the relative references.

The available controllers and the relative topics are as follows:
- _your_control_mode_ == 0, the two references are used as motor positions, topic names:
   ```
   /your_namespace/your_joint_name/reference_1
   /your_namespace/your_joint_name/reference_2
   ```
- _your_control_mode_ == 1, the two reference are used as equilibirum position and preset (according to the qbmove functions) of the actuator, topic names:
   ```
   /your_namespace/your_joint_name/equilibirum_pos
   /your_namespace/your_joint_name/preset
   ```
- _your_control_mode_ == 2, only the first reference is used as torque input for the joint (suposed rigid), while the second reference is disabled. The topic names become
   ```
   /your_namespace/your_joint_name/torque
   /your_namespace/your_joint_name/disabled
   ```

The remaining tags are used to enable other useful publishers, one for publishing the value of the elastic torques, in the following topics
   ```
   /your_namespace/your_joint_name/tau_el1_state
   /your_namespace/your_joint_name/tau_el2_state
   ```
and the other one in which retrieving useful information about the joint state, i.e., joint position and velocity _q,dq_, current references _ref_1,ref_2, joint torque _tau_ and joint stiffness _stiff_. All of these values are stored in a cusotom message on the following topic
   ```
   /your_namespace/your_joint_name/state
   ```
Finally, it is possible to apply an external torque on the joint with the following subscriber topic (provided that the _sub_ext_tau_ is set as true!)
   ```
   /your_namespace/your_joint_name/ext_tau
   ```

