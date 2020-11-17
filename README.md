# robobo-gazebo-simulator

Repository for Robobo robot simulation in the Gazebo environment.

## Requirements

* Ubuntu 16.04
* Robot Operating System - Kinetic
     * http://wiki.ros.org/ROS/Installation
     * http://wiki.ros.org/ROS/Tutorials
* Gazebo7
     * http://gazebosim.org/tutorials

## Installation

The first step is to open a terminal window and create a ROS Workspace:

```bash
mkdir –p ~/<workspace_name>/src
cd ~/<workspace_name>
catkin_make
```

This model uses the original Robobo ROS messages, so it is necessary to clone the Robobo simulator repositories (model and messages) in your own workspace:

```bash
cd ~/<workspace_name>/src
git clone https://github.com/mintforpeople/robobo-gazebo-simulator
git clone https://github.com/mintforpeople/robobo-ros-msgs
```

Next, you have to clone the Opencv-apps repository because the Robobo ROS messages package depends on it:

```bash
git clone https://github.com/ros-perception/opencv_apps
```

Finally, you have to build the workspace:

```bash
cd ~/<workspace_name>
catkin_make
```

## Model launch

You must do the first two steps in each new terminal you need to use the model:

```bash
cd ~/<workspace_name>
source devel/setup.bash
```

To launch the model:

```bash
roslaunch robobo_gazebo robobo.launch
```

As a consequence, Gazebo will open and the Robobo model will be shown, in a similar way as in the following image (depending on the selected world file):

<p align="center">
<img src="https://github.com/mintforpeople/robobo-gazebo-simulator/blob/master/gazebo.png" width=700" 
</p>

To control the robot from a Python script, please refer to the video tutorial available at:

https://documentation.theroboboproject.com/gazebo/tutorial_gazebo.mp4

From minute 5:30 onwards, there is a detailed explanation regarding it.

## ROS topics

Robobo publishes all its sensors as ROS topics. The current available sensors in simulation are:

* Infrared sensors
* Motor encoders (wheels and PAN-TILT unit)
* Inertial Measurement Unit (accelerometer, orientation)
* Ambient light
* Base battery
* Smartphone battery
* Camera image (compressed)

It is possible to access the list of topics using:

```bash
rostopic list
```
For instance, to listen the TILT position topic:

```bash
rostopic echo /robot/robobo/tilt
```

Please, refer to the ROS documentation for a detailed explanation of the Robobo sensor topics:

https://github.com/mintforpeople/robobo-programming/wiki/Robobo-Topics

## ROS services

In addition, the following actuators are available:
* Wheel motors
* PANT-TILT unit motors

It is possible to access the list of topics using:

```bash
rosservice list
```
For instance, to call the service that moves the wheel motors, we can do the following:

```bash
rosservice call /robot/robobo/moveWheels {

lspeed:
  data: 0
rspeed:
  data: 0
time:
  data: 0
unlockid:
  data: 0
error:
  data: 0
}
```
That we can edit with our specific parameters, for instance:

```bash
lspeed:
  data: 20
rspeed:
  data: 20
time:
  data: 2000
unlockid:
  data: 0
error:
  data: 0

}
```

That will make the Gazebo model move at speed 20 on each wheel during 2 seconds.

Please, refer to the ROS documentation for a detailed explanation of the Robobo actuation services:

https://github.com/mintforpeople/robobo-programming/wiki/Robobo-Services

## Simulation configuration

The simulation can be configured in many ways. First, the launch file (https://github.com/mintforpeople/robobo-gazebo-simulator/blob/master/launch/robobo.launch) allows us to set the main configuration parameters of the simulation. Specifically:

To change the simulated world, just modify "test.world" by the name of the world you want to use in the top part of the launch file:

```xml
<!--craete a new world-->
	<include file="$(find gazebo_ros)/launch/empty_world.launch">
		<!-- You can change the world name for your own -->
		<arg name="world_name" value="$(find robobo_gazebo)/worlds/test.world"/>
		<arg name="paused" value="false"/>
	</include>
```

The model description consists of five configurable parameters:

```xml
<!--define parameters-->
		<param name="tf_prefix" value="robobo_tf" />
		<arg name="robobo_name" default="robobo"/>
		<param name="robot_description" command="$(find xacro)/xacro --inorder '$(find robobo_gazebo)/urdf/robobo.urdf.xacro' 
			pusher:=true
			camera_front:=true
			emotion:=NORMAL
			visualize_irSensor:=false 
			visualize_camera:=false" />
```

1. _pusher_: determines whether or not the robot carries the pusher piece. 
     * Values: false=without pusher, true=with pusher.
2. _camera_front_: Use the front or back camera of the Smartphone.     
     * Values: false=front camera, true=back camera.
3. _emotion_: changes the Robot's facial expression that is displayed in the Smartphone's screen. 
     * Values: HAPPY, SAD, ANGRY, SMYLING, LAUGHTING, EMBARRASSED, SURPRISED, IN_LOVE, NORMAL, SLEEPING, AFRAID and TIRED.
4. _visualize_irsensor_: determines if the infrared sensors are displayed in the gazebo model.
     * Values: true, false.
5. _visualize_camera_: determines if a camera preview is displayed in gazebo.
     * Values: true, false.


## Model configuration

The robot model can be configured through the file robobo.urdf.xacro (https://github.com/mintforpeople/robobo-gazebo-simulator/blob/master/urdf/robobo.urdf.xacro) which allows to adapt its response to the specific features of the user's smartphone. Specifically, the following parameters are available (in the same order as shown in the file):

1. Change the dimensions and mass of the smartphone
 
2. Change the size of the camera image and the position on the smartphone.
 
3. Change the position and field of view (fov) of the light sensor.
 
4. Change the IMU characteristics, position, noise level and offset.
 
5. Change the initial battery level of the base and smartphone.

```xml
<!--Create tilt-smartphone-->
		<xacro:tilt_link mass="0.2" width="0.0778" length="0.1581" depth="0.0077" emotion_link="$(arg emotion)"/>

		<!--Create camera sensor-->
		<xacro:camera_sensor name="front_camera" width="480" height="640" x="0.1479" y="0.011" z="0" camera="$(arg camera_front)" visualize="$(arg visualize_camera)"/>

		<!--Create light sensor-->
		<xacro:light_sensor x="0.1479" y="0.011" z="0" fov="0.3"/>

		<!--Create IMU sensor-->
		<xacro:IMU_sensor name="IMU" x="0" y="0" z="0" roll="0" pitch="0" yaw="0" noise="0">
			<xyzOffset>0 0 0</xyzOffset>
			<rpyOffset>0 0 0</rpyOffset>
		</xacro:IMU_sensor>

		<!--Define base and smartphone battery charge in percent-->
		<xacro:base_battery initialCharge="90"/>
		<xacro:phone_battery initialCharge="85"/>
```

## Advanced configuration

The file model.urdf.xacro (https://github.com/mintforpeople/robobo-gazebo-simulator/blob/master/urdf/robobo/model.urdf.xacro) contains the xacro:macro code with all the definitions and settings from the previous files. It consists of the following macros:

* xacro:macro name = "tilt_link": creates the Robot Tilt System, configuring the size of the Smartphone and the Robot's face.
* xacro:macro name = "base_battery": creates the base battery, as a parameter the initial charge in percentage.
* xacro:macro name = "phone_battery": creates the smartphone battery, as parameter the initial charge in percentage.
* xacro:macro name = "gazebo_physics_dynamic": defines the physical characteristics of the models, to perform the dynamic calculations in gazebo. 
* xacro:macro name = "infrared_link: creates a "ghost" link to place the infrared sensors.
* xacro:macro name = "infrared": creates the infrared sensors, both the one used in gazebo and the one used to visualize in rviz.
* xacro:macro name = "camera_sensor": creates the smartphone camera.
* xacro:macro name = "light_sensor": creates the light sensor.
* xacro:macro name = "IMU_sensor": creates the IMU.
* xacro:macro name = "model_plugin": runs the plugins that should be active from the beginning of the model creation.
* xacro:macro name = "robobo_link": creates the base and Pan system of the robobo, defining the joints, infrared sensors, the physics of the models and whether or not it should carry the Pusher

## World configuration

There are several world models created specifically for Robobo, that can be accessed from: https://github.com/mintforpeople/robobo-gazebo-simulator/tree/master/worlds


At this moment, the following worlds are available:

* Table: it contains a white table with cardboard boxes, cylinders, balls and signs with aruco markers.
- Warehouse: it contains areas of different colors and aruco markers, in which to store the cylinders of that color.
- City1: a small city with a straight lane and a traffic circle that lead to the same road. It has a pedestrian crossing and signs with arucos or qr markers (to choose).
- Maze1: a maze with natural lighting and a hidden green cylinder.
- Maze2: the same maze as the previous world, but the ambient light is off, and instead of a green cylinder, we have a light bulb emitting light.
- City2: A city with a traffic circle and several exits. In this case the signals have QR markers.
- City3: The same city as in "City2" but the signs have aruco markers.

In addition, different objects that can be included in these worlds have been modeled, so the robot can interact with them. They can be downloaded from: 

https://documentation.theroboboproject.com/gazebo/models_editor_models.zip

It contains a folder called _models_editor_models_. In this folder, all the models are saved in .sdf format, so that later the user can insert them into any gazebo world. To do this, the folder must be copied into the user's folder (/home/user). 

When gazebo starts, in the left side menu, press the second tab "Insert". If the models do not appear, click on the "BOX" button, located in the upper menu. Then, create a box and save it in the default folder (model_editor_models). The objects will now appear in the "Insert" tab. To put an object in the world, simply click on it and leave it in the desired position.

## Multi robot enviroment

This Gazebo package supports multi robot enviroments. To spawn a Robobo you should add a group tag like the following to the launch file (see [here](launch/robobo_multi.launch)):

```xml
	<group ns="NAME*">
		
		<!--define parameters-->
		<param name="tf_prefix" value="NAME*_tf" />
		<arg name="robobo_name" default="NAME*"/>
		<param name="robot_description" command="$(find xacro)/xacro --inorder '$(find robobo_gazebo)/urdf/robobo.urdf.xacro' 
			pusher:=true
			camera_front:=true
			emotion:=NORMAL
			visualize_irSensor:=false 
			visualize_camera:=false" />

		<!--launch robobo model-->
		<node name="robobo_model" pkg="gazebo_ros" type="spawn_model" respawn="false" output="screen" args="-urdf -x 0 -y 0 -z 0 -R 0 -P 0 -Y 0  -model $(arg robobo_name) summit -param robot_description"/>

		<!--launch infrared sensors-->
		<node name="robobo_irs" pkg="robobo_gazebo" type="robobo_irs" args="-n $(arg robobo_name)"/>

		<!--launch image node for light sensor-->
		<node name= "image_view" pkg="image_view" type= "image_view" respawn= "false" output ="screen">
			<remap from="image" to="/robot/$(arg robobo_name)/light_sensor/image_raw"/>
			<param name="autosize" value="true"/>
			<param name="image_transport" value="compressed"/>
		</node>
	</group>
```

Where *NAME* should be changed to a unique name that will help to identify each specific robot. So each time you want to execute a script that communicates with that Robobo, remember to specify the namespace; ROS allows you to change that using the private variable "*_ns*".

### Example

To launch a example script in multi-robot configuration, in addition to include the robots in the launch file, an appropriate script must be developed. 

To launch such script, the following steps should be performed:

```bash
cd robobo_ws/
source devel/setup.bash
roslaunch robobo_gazebo robobo_multi.launch
```
And in a different terminal window:

```bash
cd robobo_ws/
source devel/setup.bash
rosrun robobo_example.py
```

## Structure

This package contains the following folders:

* launch: contains the launch files used by roslaunch.
* materials: contains the facial expression of the Robobo model.
* meshes: contains the 3D models of Robobo.
* models: contains the models used by Gazebo in .stl format. 
* nodes: contains the node used to republish IR sensons data.
* plugins: contains the source code of the plugins used by the *robobo* model.
* rviz_config: contains the rviz configuration file to visualize models and sensors.
* urdf: containd the xacro configuration files.
* scripts: contains test scripts and a validation script.
* worlds: contains the predefined worlds included with the model.

## License

robobo-gazebo-simulator is available under the Apache 2.0 license. See the LICENSE file for more info.

## Acknowledgement
<!-- 
    ROSIN acknowledgement from the ROSIN press kit
    @ https://github.com/rosin-project/press_kit
-->

<a href="http://rosin-project.eu">
  <img src="http://rosin-project.eu/wp-content/uploads/rosin_ack_logo_wide.png" 
       alt="rosin_logo" height="60" >
</a>

Supported by ROSIN - ROS-Industrial Quality-Assured Robot Software Components.  
More information: <a href="http://rosin-project.eu">rosin-project.eu</a>

<img src="http://rosin-project.eu/wp-content/uploads/rosin_eu_flag.jpg" 
     alt="eu_flag" height="45" align="left" >  

This project has received funding from the European Union’s Horizon 2020  
research and innovation programme under grant agreement no. 732287. 



