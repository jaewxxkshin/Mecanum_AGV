#include <ros/ros.h>
#include <iostream>

#include "tf/transform_datatypes.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_msgs/TFMessage.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Transform.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <std_msgs/Int16MultiArray.h>
#include <std_msgs/Float32MultiArray.h>

geometry_msgs::Vector3 pos;
std_msgs::Float32MultiArray wp_x_sub;
std_msgs::Float32MultiArray wp_y_sub;

void posCallback(const geometry_msgs::Vector3& msg);
void wp_x_Callback(const std_msgs::Float32MultiArray::ConstPtr& array);
void wp_y_Callback(const std_msgs::Float32MultiArray::ConstPtr& array);

int main(int argc, char **argv)
{   
    ros::init(argc, argv,"magv_controller");

    ros::NodeHandle nh;
    ros::Rate loop_rate(1);    
    wp_x_sub.data.resize(10);
    wp_y_sub.data.resize(10);
    ros::Subscriber d435_pos=nh.subscribe("/d435_pos",100,posCallback);//ros::TransportHints().tcpNoDelay());
    ros::Subscriber waypoint_x=nh.subscribe("waypoints_x",100,wp_x_Callback);//ros::TransportHints().tcpNoDelay());
    ros::Subscriber waypoint_y=nh.subscribe("waypoints_y",100,wp_y_Callback);//ros::TransportHints().tcpNoDelay());
    
    ros::spin();
	loop_rate.sleep();
    return 0;
}

void posCallback(const geometry_msgs::Vector3& msg){
	pos.x=msg.x;
	pos.y=msg.y;
	pos.z=-msg.z;

	ROS_INFO("Translation - [x: %f  y:%f  z:%f]",pos.x,pos.y,pos.z);

	/*double dt = ((double)ros::Time::now().sec-(double)posTimer.sec)+((double)ros::Time::now().nsec-(double)posTimer.nsec)/1000000000.;
	z_vel = (pos.z-prev_z)/dt;
	posTimer = ros::Time::now();
	prev_z = pos.z;*/
	//ROS_INFO("z_vel : %f",z_vel);
}

// cout.precision(6);

void wp_x_Callback(const std_msgs::Float32MultiArray::ConstPtr& array)
{
	for (int i = 0; i < 10; i++) //[J]there might be another way not to use for.
	{
		wp_x_sub.data[i] = array->data[i];	

	}
	//[W] each channels's values
	ROS_INFO(" x_1:[%f] x_2:[%f] x_3:[%f] x_4:[%f] x_5:[%f] x_6:[%f] x_7:[%f] x_8:[%f] x_9:[%f] x_10:[%f]", wp_x_sub.data[0], wp_x_sub.data[1], wp_x_sub.data[2], wp_x_sub.data[3],wp_x_sub.data[4],wp_x_sub.data[5], wp_x_sub.data[6], wp_x_sub.data[7],wp_x_sub.data[8], wp_x_sub.data[9]);
	
}


void wp_y_Callback(const std_msgs::Float32MultiArray::ConstPtr& array)
{
	for (int i = 0; i < 10; i++) //[J]there might be another way not to use for.
	{
		wp_y_sub.data[i] = array->data[i];	

	}
	//[W] each channels's values
	ROS_INFO(" y_1:[%f] y_2:[%f] y_3:[%f] y_4:[%f] y_5:[%f] y_6:[%f] y_7:[%f] y_8:[%f] y_9:[%f] y_10:[%f]",
     wp_y_sub.data[0], wp_y_sub.data[1], wp_y_sub.data[2], wp_y_sub.data[3],wp_y_sub.data[4],wp_y_sub.data[5], wp_y_sub.data[6], wp_y_sub.data[7],wp_y_sub.data[8], wp_y_sub.data[9]);
}