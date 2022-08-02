#include <ros/ros.h>
#include <iostream>
#include <cmath>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

#include "tf/transform_datatypes.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_msgs/TFMessage.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Transform.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <std_msgs/Int16MultiArray.h>
#include <std_msgs/Float32MultiArray.h>


#include "nav_msgs/Odometry.h"

double temp1, temp2;
double c, s;

geometry_msgs::Vector3 pos;
std_msgs::Float32MultiArray wp_x_sub;
std_msgs::Float32MultiArray wp_y_sub;

geometry_msgs::Quaternion rot;
// geometry_msgs::Vector3 t265_lin_vel;
// geometry_msgs::Vector3 t265_ang_vel;

// Eigen::Vector3d cam_v;

Eigen::Vector3d cam_att;
std::vector<double> wp_set;
std::vector<std::vector<double>> wp_all;

std::vector<double> wp_r_x;
std::vector<double> wp_r_y;

void rotCallback(const geometry_msgs::Quaternion& msg);
void posCallback(const geometry_msgs::Vector3& msg);
void wp_x_Callback(const std_msgs::Float32MultiArray::ConstPtr& array);
void wp_y_Callback(const std_msgs::Float32MultiArray::ConstPtr& array);
void t265OdomCallback(const nav_msgs::Odometry::ConstPtr& msg);
void vec_delete_double(std::vector<double> &vec);

ros::Publisher linear_velocity;

int main(int argc, char **argv)
{   
    ros::init(argc, argv,"magv_controller");

    ros::NodeHandle nh;
    ros::Rate loop_rate(1);    
    wp_x_sub.data.resize(10);
    wp_y_sub.data.resize(10);


	ros::Subscriber d435_rot=nh.subscribe("/d435_rot",100,rotCallback);
	ros::Subscriber d435_pos=nh.subscribe("/d435_pos",100,posCallback);//ros::TransportHints().tcpNoDelay());
    ros::Subscriber waypoint_x=nh.subscribe("waypoints_x",100,wp_x_Callback);//ros::TransportHints().tcpNoDelay());
    ros::Subscriber waypoint_y=nh.subscribe("waypoints_y",100,wp_y_Callback);//ros::TransportHints().tcpNoDelay());
	ros::Subscriber t265_odom=nh.subscribe("/camera/odom/sample",100,t265OdomCallback,ros::TransportHints().tcpNoDelay());

    ros::spin();
	loop_rate.sleep();
    return 0;
}

void posCallback(const geometry_msgs::Vector3& msg){
	pos.x=msg.x;
	pos.y=msg.y;
	pos.z=-msg.z;

	// ROS_INFO("Translation - [x: %f  y:%f  z:%f]",pos.x,pos.y,pos.z);

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
	// ROS_INFO(" x_1:[%f] x_2:[%f] x_3:[%f] x_4:[%f] x_5:[%f] x_6:[%f] x_7:[%f] x_8:[%f] x_9:[%f] x_10:[%f]", wp_x_sub.data[0], wp_x_sub.data[1], wp_x_sub.data[2], wp_x_sub.data[3],wp_x_sub.data[4],wp_x_sub.data[5], wp_x_sub.data[6], wp_x_sub.data[7],wp_x_sub.data[8], wp_x_sub.data[9]);
	
}


void wp_y_Callback(const std_msgs::Float32MultiArray::ConstPtr& array)
{
	for (int i = 0; i < 10; i++) //[J]there might be another way not to use for.
	{
		wp_y_sub.data[i] = array->data[i];	

	}
	//[W] each channels's values
	// ROS_INFO(" y_1:[%f] y_2:[%f] y_3:[%f] y_4:[%f] y_5:[%f] y_6:[%f] y_7:[%f] y_8:[%f] y_9:[%f] y_10:[%f]",
    //  wp_y_sub.data[0], wp_y_sub.data[1], wp_y_sub.data[2], wp_y_sub.data[3],wp_y_sub.data[4],wp_y_sub.data[5], wp_y_sub.data[6], wp_y_sub.data[7],wp_y_sub.data[8], wp_y_sub.data[9]);
}

void rotCallback(const geometry_msgs::Quaternion& msg){
	rot.x=msg.x;
	rot.y=msg.y;
	rot.z=msg.z;
	rot.w=msg.w;
	// ROS_INFO("norm of quat : [%f]", pow(msg.x,2)+pow(msg.y,2)+pow(msg.z,2)+pow(msg.w,2));
	/*t265_att.x=atan2((rot.y*rot.z+rot.w*rot.x),1.-2.*(rot.z*rot.z+rot.w*rot.w));
	t265_att.y=asin(2.*(rot.y*rot.w-rot.x*rot.z));
	t265_att.z=atan2((rot.y*rot.x+rot.z*rot.w),1.-2.*(rot.w*rot.w+rot.x*rot.x));*/
	
	// tf::Quaternion quat;
	// tf::quaternionMsgToTF(rot,quat);
	// tf::Matrix3x3(quat).getRPY(t265_att.x,t265_att.y,t265_att.z);
	// ROS_INFO("Attitude - [r: %f  p: %f  y:%f]",t4_att(0),t4_att(1),t4_att(2));	
}



void t265OdomCallback(const nav_msgs::Odometry::ConstPtr& msg){
	// t265_lin_vel=msg->twist.twist.linear;
	// t265_ang_vel=msg->twist.twist.angular;
	// t265_quat=msg->pose.pose.orientation;
	tf::Quaternion quat;
	tf::quaternionMsgToTF(rot,quat);
	tf::Matrix3x3(quat).getRPY(cam_att(0),cam_att(1),cam_att(2));
	// cam_v << t265_lin_vel.x, t265_lin_vel.y, t265_lin_vel.z;
	// R_v << cos(pi/4.), -sin(pi/4.),  0.,
 	//        sin(pi/4.),  cos(pi/4.),  0.,
	//                 0.,           0.,  1.;

	// v = R_v*cam_v;
    //     //t4_att = R_a*cam_att;
	// lin_vel.x=v(0);
	// lin_vel.y=v(1);
	// lin_vel.z=-v(2);
	// ROS_INFO("Attitude - [r: %f  p: %f  y:%f]",cam_att(0),cam_att(1),cam_att(2));
	//ROS_INFO("Linear_velocity - [x: %f  y: %f  z:%f]",v(0),v(1),v(2));
	//ROS_INFO("Angular_velocity - [x: %f  y: %f  z:%f]",w(0),w(1),w(2));
}

void vec_delete_double(std::vector<double> &vec)
{
    vec.clear();
    std::vector<double>().swap(vec);
}

// void cal_rot_wp()
// {	
// 	c = cos(cam_att(2));
// 	s = sin(cam_att(2));
// 	for(int i = 0; i < 10; i++)
// 	{
// 		temp1 = c*wp_x_sub.data[i] - s*wp_y_sub.data[i];
// 		temp2 = s*wp_x_sub.data[i] + c*wp_y_sub.data[i];
// 		wp_r_x.push_back(temp1);
// 		wp_r_y.push_back(temp2);
// 	}
// 	std::cout <<
// }

