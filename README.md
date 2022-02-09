# PX4_Development

#### The repositiry contains all the source code and other dependent files which I created as part of my PX4 development

>PX4 is the Professional Autopilot. Developed by world-class developers from industry and academia, and supported by an active world wide community, it powers all kinds of vehicles from racing and cargo drones through to ground vehicles and submersibles.

![](https://github.com/rafism1997/PX4_Development/blob/main/px4.png)


## Creating Workspace : 

# 1. Make a Directory

```bash
mkdir workspace && cd workspace && mkdir src
```

# 2. Clone repositories into src directory

```bash
cd src
git clone <url of repo>
```

 If you want to binary install

- use the name of your ros distro - noetic or melodic

```bash
sudo apt install ros-{distro}-mavros
sudo apt install ros-{distro}-mavros-extras
```

To install octomap package

```bash
sudo apt-get install ros-{distro}-octomap
```

To install Eigen library

```bash
sudo apt install libeigen3-dev
```

- Go back to main workspace and build the ros workspace

```bash
catkin_make  
```

or

```bash
catkin build
```

- To use catkin build, “catkin tools” need to be installed

# 3. To make your own package

Template

```bash
catkin_create_pkg package_name {roscpp mavros rospy geometry_msgs .. all dependencies}
```

```bash
catkin_create_pkg copter roscpp mavros rospy geometry_msgs nav_msgs 
```

# 4. CMakeLists.txt

```xml
cmake_minimum_required(VERSION 3.0.2)
project(package_name)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  mavros
  mavros_msgs
  octomap_msgs
  octomap_ros
  pcl_ros
  roscpp
  rospy
  sensor_msgs
  tf
  std_msgs
  visualization_msgs
)

find_package(OpenCV REQUIRED)
find_package(PkgConfig)
pkg_search_module(Eigen3 REQUIRED eigen3)
find_package(octomap REQUIRED)

include_directories(include ${catkin_INCLUDE_DIRS})
include_directories(${EIGEN_INCLUDE_DIRS})
include_directories(${OCTOMAP_INCLUDE_DIRS})
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES FastPlannerOctomap
#  CATKIN_DEPENDS geometry_msgs mavros mavros_msgs octomap octomap_msgs pcl_ros roscpp rospy sensor_msgs std_msgs visualization_msgs
#  DEPENDS system_lib
)

# add all the files to be compiled
#name of the cpp fiel
add_executable(node_name src/node_name.cpp)
target_link_libraries(node_name ${catkin_LIBRARIES})

target_link_libraries(${OCTOMAP_LIBRARIES})

```

# Package xml

```xml
<?xml version="1.0"?>
<package format="2">
  <name>FastPlannerOctomap</name>
  <version>0.0.0</version>
  <description>The FastPlannerOctomap package</description>

  <!-- One maintainer tag required, multiple allowed, one person per tag -->
  <!-- Example:  -->
  <!-- <maintainer email="jane.doe@example.com">Jane Doe</maintainer> -->
<maintainer email="traveller@todo.todo">traveller</maintainer>

  <!-- One license tag required, multiple allowed, one license per tag -->
  <!-- Commonly used license strings: -->
  <!--   BSD, MIT, Boost Software License, GPLv2, GPLv3, LGPLv2.1, LGPLv3 -->
  <license>TODO</license>

  <!-- Url tags are optional, but multiple are allowed, one per tag -->
  <!-- Optional attribute type can be: website, bugtracker, or repository -->
  <!-- Example: -->
  <!-- <url type="website">http://wiki.ros.org/FastPlannerOctomap</url> -->

  <!-- Author tags are optional, multiple are allowed, one per tag -->
  <!-- Authors do not have to be maintainers, but could be -->
  <!-- Example: -->
  <!-- <author email="jane.doe@example.com">Jane Doe</author> -->

  <!-- The *depend tags are used to specify dependencies -->
  <!-- Dependencies can be catkin packages or system dependencies -->
  <!-- Examples: -->
  <!-- Use depend as a shortcut for packages that are both build and exec dependencies -->
  <!--   <depend>roscpp</depend> -->
  <!--   Note that this is equivalent to the following: -->
  <!--   <build_depend>roscpp</build_depend> -->
  <!--   <exec_depend>roscpp</exec_depend> -->
  <!-- Use build_depend for packages you need at compile time: -->
  <!--   <build_depend>message_generation</build_depend> -->
  <!-- Use build_export_depend for packages you need in order to build against this package: -->
  <!--   <build_export_depend>message_generation</build_export_depend> -->
  <!-- Use buildtool_depend for build tool packages: -->
  <!--   <buildtool_depend>catkin</buildtool_depend> -->
  <!-- Use exec_depend for packages you need at runtime: -->
  <!--   <exec_depend>message_runtime</exec_depend> -->
  <!-- Use test_depend for packages you need only for testing: -->
  <!--   <test_depend>gtest</test_depend> -->
  <!-- Use doc_depend for packages you need only for building documentation: -->
  <!--   <doc_depend>doxygen</doc_depend> -->
  <buildtool_depend>catkin</buildtool_depend>
  <build_depend>geometry_msgs</build_depend>
  <build_depend>mavros</build_depend>
  <build_depend>mavros_msgs</build_depend>
  <build_depend>octomap_ros</build_depend>
  <build_depend>octomap_msgs</build_depend>
  <build_depend>pcl_ros</build_depend>
  <build_depend>roscpp</build_depend>
  <build_depend>rospy</build_depend>
  <build_depend>sensor_msgs</build_depend>
  <build_depend>std_msgs</build_depend>
  <build_depend>visualization_msgs</build_depend>
  <build_export_depend>geometry_msgs</build_export_depend>
  <build_export_depend>mavros</build_export_depend>
  <build_export_depend>mavros_msgs</build_export_depend>
  <build_export_depend>octomap_ros</build_export_depend>
  <build_export_depend>octomap_msgs</build_export_depend>
  <build_export_depend>pcl_ros</build_export_depend>
  <build_export_depend>roscpp</build_export_depend>
  <build_export_depend>rospy</build_export_depend>
  <build_export_depend>sensor_msgs</build_export_depend>
  <build_export_depend>std_msgs</build_export_depend>
  <build_export_depend>visualization_msgs</build_export_depend>
  <exec_depend>geometry_msgs</exec_depend>
  <exec_depend>mavros</exec_depend>
  <exec_depend>mavros_msgs</exec_depend>
  <exec_depend>octomap_ros</exec_depend>
  <exec_depend>octomap_msgs</exec_depend>
  <exec_depend>pcl_ros</exec_depend>
  <exec_depend>roscpp</exec_depend>
  <exec_depend>rospy</exec_depend>
  <exec_depend>sensor_msgs</exec_depend>
  <exec_depend>std_msgs</exec_depend>
  <exec_depend>visualization_msgs</exec_depend>

  <!-- The export tag contains other, unspecified, tags -->
  <export>
    <!-- Other tools can request additional information be placed here -->

  </export>
</package>
```
# Create the node in your src folder

```bash
/**
 * @file offb_node.cpp
 * @brief Offboard control example node, written with MAVROS version 0.19.x, PX4 Pro Flight
 * Stack and tested in Gazebo SITL
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 2;

    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();

    while(ros::ok()){
        if( current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0))){
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent){
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        } else {
            if( !current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0))){
                if( arming_client.call(arm_cmd) &&
                    arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }

        local_pos_pub.publish(pose);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
```
- Go back to main workspace and build the ros workspace

```bash
catkin_make  
```

or

```bash
catkin build
```
