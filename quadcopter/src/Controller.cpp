// OFFBOARD POSITION CONTROL TO MANEUVER THE DRONE 

/** ROS HEADERS **/
#include<ros/ros.h>
#include<geometry_msgs/PoseStamped.h>
#include<std_msgs/Int8.h>
#include<std_msgs/Float64.h>
#include<geometry_msgs/Twist.h>
#include<mavros_msgs/State.h>
#include<mavros_msgs/SetMode.h>
#include<mavros_msgs/PositionTarget.h>
#include<mavros_msgs/CommandBool.h>
#include<mavros_msgs/CommandTOL.h>

/** PATH **/
#include<nav_msgs/Path.h>

#include<Eigen/Dense>

#include<iostream>
#include<vector>
#include<math.h>
#include<iterator>
#include<cmath>


geometry_msgs::PoseStamped way_pose; //this receives the waypoint every time subscriber is called

Eigen::Vector3d currPose;

int count;
bool WaypointUpdated = 0;
/** waypoint callback **/

void waypoint_cb(const geometry_msgs::PoseStamped pose)
{
    way_pose = pose;    
    //std::cout<<"Pose values"<<pose<<std::endl;
    WaypointUpdated = 1;
}
/** drone pose callback **/
void local_pose_cb(const geometry_msgs::PoseStamped pose)
{
    currPose(0) = pose.pose.position.x;
    currPose(1) = pose.pose.position.y;
    currPose(2) = pose.pose.position.z;
}

/** MAVROS OFFBOARD Control **/
void control(ros::Publisher pub, ros::Rate rate)
{   
    geometry_msgs::PoseStamped waypoint = way_pose;   
         
    int x[4] = {1,4,4,1};
    int y[4] = {1,1,4,4};

    for(int i=0;i< sizeof(x) / sizeof (x[0]) && i < sizeof(y) / sizeof(y[0]);i++){
        way_pose.pose.position.x = x[i];
        way_pose.pose.position.y = y[i];
    }
  
      std::cout<<"Published point "<<"X"<<way_pose.pose.position.x << "y"<< way_pose.pose.position.y <<"z"<< way_pose.pose.position.z <<std::endl;
            pub.publish(waypoint);
            rate.sleep();
      
    
}



int main(int argc, char** argv)
{
    ros::init(argc, argv, "drone_control");
    ros::NodeHandle n;

    ros::Subscriber path = n.subscribe<geometry_msgs::PoseStamped>("/waypoint",1, waypoint_cb);
    ros::Subscriber loc = n.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose",10,local_pose_cb);

    ros::Publisher wp_pub = n.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local",10);

    /* Services */
    ros::ServiceClient  arming_client = n.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    ros::ServiceClient  landing_client = n.serviceClient<mavros_msgs::CommandTOL>("mavros/cmd/landing");
    ros::ServiceClient  set_mode_client = n.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
    
    ros::Rate rate(20);
    
    
    std::cout<<"Enter  waypoint"<<std::endl;
while(!WaypointUpdated || !ros::ok())
    {
        std::cout<<"Waiting for Waypoint"<< ros::ok()<<std::endl;
        ros::spinOnce();
        rate.sleep();

        if(!ros::ok())
            {
                break;
            }
    }

   

    if(WaypointUpdated)
    {
        geometry_msgs::PoseStamped initPose;

        if(count==0)
        {
            std::cout<<"Controller started ... "<<std::endl;

            initPose.pose.position.x = 0;
            initPose.pose.position.y = 0;
            initPose.pose.position.z = 0;

            for(int i = 0; i<10 && ros::ok(); i++)
            {
                wp_pub.publish(initPose);
                ros::spinOnce();
                rate.sleep();
            }

            mavros_msgs::SetMode offb_set_mode;
            offb_set_mode.request.custom_mode = "OFFBOARD";

            mavros_msgs::CommandBool arm_cmd;
            arm_cmd.request.value = true;

        /** set mode to offboard **/
            if(set_mode_client.call(offb_set_mode) &&
                    offb_set_mode.response.mode_sent)
                {
                        ROS_INFO("Offboard enabled");
                }

        }

        count++;
        while(ros::ok())
        {
            control(wp_pub, rate);     
            ros::spinOnce();

            if(!ros::ok())
            break;

        }

    
    }

    ros::spinOnce();
    return 0;
}
