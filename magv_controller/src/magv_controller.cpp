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


//~~~~~~~~variable
double temp1, temp2;
double c, s;
double t265_px, t265_py;

geometry_msgs::Vector3 pos;
geometry_msgs::Quaternion rot;

std_msgs::Float32MultiArray wp_set_sub;

std::vector<double> wp_r_x;
std::vector<double> wp_r_y;

bool flag = 0;
// control variable
double e_x, e_y, e_psi;
double p_x, i_x, d_x;
double p_y, i_y, d_y;
double p_psi, i_psi, d_psi;
double distance = 1.;
double threshold = 0.03;
int idx = 0;

Eigen::Vector3d cam_att;

// Function 
// ----------------------------------------------------------------------------
void rotCallback(const geometry_msgs::Quaternion& msg);
void posCallback(const geometry_msgs::Vector3& msg);
void wp_set_Callback(const std_msgs::Float32MultiArray::ConstPtr& array);
void t265OdomCallback(const nav_msgs::Odometry::ConstPtr& msg);
void vec_delete_double(std::vector<double> &vec);
void cal_rot_wp();
void pos_ctrl();
// ----------------------------------------------------------------------------

int main(int argc, char **argv)
{   
    ros::init(argc, argv,"magv_controller");
    ros::NodeHandle nh;
    ros::Rate loop_rate(1);    
	wp_set_sub.data.resize(20);

	ros::Subscriber d435_rot=nh.subscribe("/d435_rot",100,rotCallback);
	ros::Subscriber d435_pos=nh.subscribe("/d435_pos",100,posCallback);//ros::TransportHints().tcpNoDelay());
	ros::Subscriber waypoint_set=nh.subscribe("waypoints_set",100,wp_set_Callback);//ros::TransportHints().tcpNoDelay());
    ros::Subscriber t265_odom=nh.subscribe("/camera/odom/sample",100,t265OdomCallback,ros::TransportHints().tcpNoDelay());

	ros::spin();

	loop_rate.sleep();
    return 0;
}

void posCallback(const geometry_msgs::Vector3& msg){
	pos.x=msg.x;
	pos.y=msg.y;
	pos.z=-msg.z;
	if(flag == 1) pos_ctrl();
	// ROS_INFO("Translation - [x: %f  y:%f  z:%f]",pos.x,pos.y,pos.z);
}


void wp_set_Callback(const std_msgs::Float32MultiArray::ConstPtr& array)
{
	for (int i = 0; i < 20; i++) 
	{
		wp_set_sub.data[i] = array->data[i];	
	}
	t265_px = pos.x;
	t265_py = pos.y;
	cal_rot_wp();
	flag = 1;
	// idx will initialized when new waypoint is created
	std::cout << " waypoint is created now! " <<std::endl;
	idx = 0;
}


void rotCallback(const geometry_msgs::Quaternion& msg){
	rot.x=msg.x;
	rot.y=msg.y;
	rot.z=msg.z;
	rot.w=msg.w;
	// ROS_INFO("norm of quat : [%f]", pow(msg.x,2)+pow(msg.y,2)+pow(msg.z,2)+pow(msg.w,2));
}

void t265OdomCallback(const nav_msgs::Odometry::ConstPtr& msg){
	// t265_lin_vel=msg->twist.twist.linear;
	// t265_ang_vel=msg->twist.twist.angular;
	// t265_quat=msg->pose.pose.orientation;
	tf::Quaternion quat;
	tf::quaternionMsgToTF(rot,quat);
	tf::Matrix3x3(quat).getRPY(cam_att(0),cam_att(1),cam_att(2));
	// std::cout << " =====created camera att 2 : " << cam_att(2) << std::endl;
}

void vec_delete_double(std::vector<double> &vec)
{
    vec.clear();
    std::vector<double>().swap(vec);
}

void cal_rot_wp()
{	
	vec_delete_double(wp_r_x);
	vec_delete_double(wp_r_y);
	c = cos(cam_att(2));
	s = sin(cam_att(2));
	for(int i = 0; i < 10; i++)
	{
		temp1 = c*wp_set_sub.data[2*i] - s*wp_set_sub.data[2*i+1] + t265_px;
		temp2 = s*wp_set_sub.data[2*i] + c*wp_set_sub.data[2*i+1] + t265_py;
		wp_r_x.push_back(temp1);
		wp_r_y.push_back(temp2);
		//std::cout << "wp_r_x: " << wp_r_x[i] << " wp_r_y: " << wp_r_y[i]  << std::endl;
	}
	//std::cout <<"s" << std::endl;
}

void pos_ctrl()
{
	if( distance < threshold) idx += 1;
	//std::cout << idx << std::endl;
	distance = sqrt(pow((wp_r_x[idx]-pos.x),2)+pow((wp_r_y[idx]-pos.y),2));
	e_psi = atan2((wp_r_y[idx+1]-wp_r_y[idx]),(wp_r_x[idx+1]-wp_r_x[idx]))-(cam_att(2) + M_PI/2);
	// std::cout << " e_psi : " << e_psi << std::endl;
	
}
