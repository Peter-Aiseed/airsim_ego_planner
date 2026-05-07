# AirSim Ego-Planner Setup

This repository provides a ROS-based solution for integrating Microsoft AirSim with point cloud concatenation tools, allowing you to merge up to four depth image streams into a single unified point cloud.

## 📋 System Requirements
* **OS:** Ubuntu 20.04
* **ROS:** Noetic
* **Build Tool:** `catkin_tools`

---

## 🛠 Installation

### 1. Install AirSim ROS Wrapper
Follow the official instructions to install the AirSim ROS environment:
[AirSim ROS Packages Documentation](https://microsoft.github.io/AirSim/airsim_ros_pkgs/)

Change the AirSim/ros/src/airsim_ros_pkgs/launch/airsim_node.launch to this:
```json
<launch>
	<param name="use_sim_time" value="true"/>
	<arg name="output" default="screen"/>
	<arg name="publish_clock" default="false"/>
	<arg name="is_vulkan" default="true"/>
	<arg name="host" default="localhost" />

	<node name="airsim_clock_publisher" pkg="ego_planner" type="airsim_clock_publisher.py" output="screen" />

	<node name="airsim_node" pkg="airsim_ros_pkgs" type="airsim_node" output="$(arg output)">
		<param name="is_vulkan" type="bool" value="false" /> 
		<!-- ROS timer rates. Note that timer callback will be processed at maximum possible rate, upperbounded by the following ROS params -->
		<param name="update_airsim_img_response_every_n_sec" type="double" value="0.05" /> 
		<param name="update_airsim_control_every_n_sec" type="double" value="0.01" />
		<param name="update_lidar_every_n_sec" type="double" value="0.01" />
		<param name="publish_clock" type="bool" value="$(arg publish_clock)" />
		<param name="host_ip" type="string" value="$(arg host)" />
	</node>

	<!-- Static transforms -->
	<include file="$(find airsim_ros_pkgs)/launch/static_transforms.launch"/>
</launch>
```

### 2. Workspace Setup
Create a new workspace (or navigate to your existing one) and clone the necessary repositories into the `src` directory:

```bash
# Navigate to your workspace src
cd ~/airsim_ego_ws/src

# Clone this repository
git clone https://github.com/Peter-Aiseed/airsim_ego_planner.git

# Clone the pointcloud_concatenate repository
git clone https://github.com/aseligmann/pointcloud_concatenate.git
```

### 3. Build the Workspace
Return to the root of your workspace and compile the packages:

```bash
cd ~/airsim_ego_ws
catkin build
```

### 4. Configuration

To handle the multi-depth image setup, you must update the `pointcloud_concatenate` launch file. 

Modify `src/pointcloud_concatenate/launch/concat.launch` with the following configuration:

```json
<launch>
  <arg name="target_frame"/>
  <arg name="hz"/>
  <arg name="cloud_in1" />
  <arg name="cloud_in2"/>
  <arg name="cloud_in3"/>
  <arg name="cloud_in4" />
  <arg name="cloud_out"/>

  <node pkg="pointcloud_concatenate" type="pointcloud_concatenate_node" name="pointcloud_concat" output="log">
    <param name="target_frame" value="$(arg target_frame)" />
    <param name="clouds" value="4" />
    <param name="hz" value="$(arg hz)" />
    <remap from="cloud_in1" to="$(arg cloud_in1)" />
    <remap from="cloud_in2" to="$(arg cloud_in2)" />
    <remap from="cloud_in3" to="$(arg cloud_in3)" />
    <remap from="cloud_in4" to="$(arg cloud_in4)" />
    <remap from="cloud_out" to="$(arg cloud_out)" />
  </node>
</launch>
```
### 5. Environment Sourcing

To ensure all nodes and messages are recognized, source the ROS environment and your workspaces into bashrc:

```bash
# Source ROS Noetic
source /opt/ros/noetic/setup.bash

# Source AirSim Wrapper
source ~/AirSim/ros/devel/setup.bash

# Source project workspace
source ~/airsim_ego_ws/devel/setup.bash
```

## 🚀 Running the Simulation

### 1. Configure AirSim Settings
Before launching, update your `settings.json` (typically located in `~/Documents/AirSim`) to define the drone and its four directional cameras:

```json
{
  "SettingsVersion": 1.2,
  "SimMode": "Multirotor",
  "ClockSpeed": 0.0667,
  "LocalHostIp": "0.0.0.0",
  "Vehicles": {
    "drone_0": {
      "VehicleType": "SimpleFlight",
      "Cameras": {
        "cube_front": {
          "CaptureSettings": [{ "ImageType": 1, "Width": 160, "Height": 120, "FOV_Degrees": 90, "MotionBlurAmount": 0 }],
          "X": 0.25, "Y": 0, "Z": -0.25, "Pitch": 0, "Roll": 0, "Yaw": 0
        },
        "cube_right": {
          "CaptureSettings": [{ "ImageType": 1, "Width": 160, "Height": 120, "FOV_Degrees": 90, "MotionBlurAmount": 0 }],
          "X": 0, "Y": 0.25, "Z": -0.25, "Pitch": 0, "Roll": 0, "Yaw": 90
        },
        "cube_back": {
          "CaptureSettings": [{ "ImageType": 1, "Width": 160, "Height": 120, "FOV_Degrees": 90, "MotionBlurAmount": 0 }],
          "X": -0.25, "Y": 0, "Z": -0.25, "Pitch": 0, "Roll": 0, "Yaw": 180
        },
        "cube_left": {
          "CaptureSettings": [{ "ImageType": 1, "Width": 160, "Height": 120, "FOV_Degrees": 90, "MotionBlurAmount": 0 }],
          "X": 0, "Y": -0.25, "Z": -0.25, "Pitch": 0, "Roll": 0, "Yaw": -90
        }
      }
    }
  }
}
```
(The settings can be change, even more camera but make sure the original 4 camera name, 4 camera ImageType must have 1 (DepthPlanar), VehicleType and Vehicle name the same.)

### 2. Launch AirSim
Then launch the Airsim with this settings.json. 

## 🚀 Execution Steps

Open a separate terminal for each of the following steps.

### 1. Start ROS Master
It is recommended to run `roscore` manually first to ensure the ROS core is independent and not accidentally terminated by closing other launch files or nodes.

**Terminal 1 (Roscore):**
```bash
roscore
```

### 2. Launch AirSim ROS Wrapper
**Terminal 2 (Wrapper):**
```bash
roslaunch airsim_ros_pkgs airsim_node.launch
```
**Connection Check:** Look for the following output to confirm a successful handshake:
```
Client Ver:1 (Min Req:1), Server Ver:1 (Min Req:1)
Waiting for connection - 
Connected!

Client Ver:1 (Min Req:1), Server Ver:1 (Min Req:1)
Waiting for connection - 
Connected!

Client Ver:1 (Min Req:1), Server Ver:1 (Min Req:1)
AirsimROSWrapper Initialized!
```

*Note: If the terminal hangs at `Connected!` and doesn't show `Initialized!`, check if another AirSim process is running in the background.*

### 3. Launch Communication Bridge & Pointcloud Fusion
The bridge will trigger the drone to **take off**. The fusion node will combine the 4 depth images and automatically open **RViz**.

**Terminal 3 (Bridge):**
```bash
roslaunch ego_planner airsim_preparation.launch 
```

### 4. Final Planner Launch
**Wait!** Before running this final command, verify:
1. The preparation terminal displays: `Ego-planner to Airsim Bridge Started`.
2. RViz is open and the pointcloud is visible.

**Terminal 4 (Planner):**
Once verified, run:
```bash
roslaunch ego_planner run_in_sim.launch
```