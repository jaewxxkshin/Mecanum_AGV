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
#include <std_msgs/Int16.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int16MultiArray.h>
#include <std_msgs/Float32MultiArray.h>
#include "nav_msgs/Odometry.h"
#include <std_msgs/Bool.h>

#define const_vel 10
#define L 0.43
#define wp_num 10

std_msgs::Float32MultiArray wp_set_sub;
std_msgs::Float32MultiArray arr_psi;
std_msgs::Int16 idx_ros;

geometry_msgs::Vector3 pos;
geometry_msgs::Quaternion rot;
geometry_msgs::Vector3 t265_ang_vel;
geometry_msgs::Vector3 t265_att;

std::vector<float> wp_r_x;
std::vector<float> wp_r_y;

bool corner_flag = false;
bool flag = 0;

// control variable
int p_psi = 5;
double cmd_psi = 0;
double distance = 1.;
double threshold = 0.02; //need to change
double freq = 30;//controller loop frequency
float err_psi = 0;
float err_psi_1 = 0;
float err_psi_2 = 0;
float err_psi_dot = 0;
float des_psi_1 = 0;
float des_psi_2 = 0;
float cur_psi = 0;
float k = 0.1;
float x_ic, y_ic = 0;
// jh 
double gradient =0.0;
double temp_y = 0.0;
int16_t idx = 0;
// Function 
// ----------------------------------------------------------------------------
void rotCallback(const geometry_msgs::Quaternion& msg);
void posCallback(const geometry_msgs::Vector3& msg);
void wp_r_x_Callback(const std_msgs::Float32MultiArray::ConstPtr& array);
void wp_r_y_Callback(const std_msgs::Float32MultiArray::ConstPtr& array);
void originCallback(const std_msgs::Float32MultiArray::ConstPtr& array);
void vec_delete_float(std::vector<float> &vec);
void cal_rot_wp();
void yaw_ctrl();
void corner_decision_Callback(const std_msgs::Bool::ConstPtr& decision);
// ----------------------------------------------------------------------------

int main(int argc, char **argv)
{   
    ros::init(argc, argv,"magv_controller");
    ros::NodeHandle nh;  

	wp_set_sub.data.resize(wp_num*2);
	arr_psi.data.resize(1);

	ros::Publisher pub_psi = nh.advertise<std_msgs::Float32MultiArray>("pub_psi", 1000);
	ros::Publisher pub_idx = nh.advertise<std_msgs::Int16>("pub_idx", 1000);

	ros::Subscriber d435_rot=nh.subscribe("/d435_rot",100,rotCallback);
	ros::Subscriber d435_pos=nh.subscribe("/d435_pos",100,posCallback);
	ros::Subscriber waypoint_r_x_sub =nh.subscribe("wp_r_x",100,wp_r_x_Callback);
    ros::Subscriber waypoint_r_y_sub =nh.subscribe("wp_r_y",100,wp_r_y_Callback);
	ros::Subscriber corner_decision_sub=nh.subscribe("/corner_decision",100,corner_decision_Callback);
	ros::Subscriber d435_origin_sub =nh.subscribe("/d435_origin",100,originCallback);
	
	ros::Rate loop_rate(100);
	
	while (ros::ok()) 
	{
		pub_psi.publish(arr_psi);
		pub_idx.publish(idx_ros);
		ros::spinOnce();
		loop_rate.sleep();
	}
    return 0;
}


void posCallback(const geometry_msgs::Vector3& msg){
	pos.x=msg.x;
	pos.y=msg.y;
	pos.z=-msg.z;
	if(flag == 1) yaw_ctrl();
	// ROS_INFO("Translation - [x: %f  y:%f  z:%f]",pos.x,pos.y,pos.z);
}


void wp_r_x_Callback(const std_msgs::Float32MultiArray::ConstPtr& array)
{
	vec_delete_float(wp_r_x);
	for (int i = 0; i < wp_num; i++) 
	{
		wp_r_x.push_back(array->data[i]);	
	}

	flag = 1;

	// idx will initialized when new waypoint is created
	idx = 0;
}

void wp_r_y_Callback(const std_msgs::Float32MultiArray::ConstPtr& array)
{
	vec_delete_float(wp_r_y);
	for (int i = 0; i < wp_num; i++) 
	{
		wp_r_y.push_back(array->data[i]);	
	}
	//std::cout << "test: " << wp_r_y[9] << std::endl;
}

void originCallback(const std_msgs::Float32MultiArray::ConstPtr& array)
{
	x_ic = array->data[0];
	y_ic = array->data[1];
	std::cout << "x : " << x_ic << "y : " << y_ic << std::endl;
}


void rotCallback(const geometry_msgs::Quaternion& msg)
{
	rot.x=msg.x;
	rot.y=msg.y;
	rot.z=msg.z;
	rot.w=msg.w;

	tf::Quaternion quat;
	tf::quaternionMsgToTF(rot,quat);
	tf::Matrix3x3(quat).getRPY(t265_att.x,t265_att.y,t265_att.z);	
}

void vec_delete_float(std::vector<float> &vec)
{
    vec.clear();
    std::vector<float>().swap(vec);
}

void yaw_ctrl()
{
	gradient = -(wp_r_x[1]-wp_r_x[0])/(wp_r_y[1]-wp_r_y[0]);
	// f1(x)
	double tmp_x = pos.x;
	double tmp_y = pos.y;
	temp_y = gradient *(tmp_x - wp_r_x[idx])+ wp_r_y[idx];
	// f2(x)
	// y2 = gradient *(pos.x - wp_r_x[index+1])+ wp_r_y[index+1];

	if ( tmp_y > temp_y) idx++;
	std::cout << " index : " << idx << std::endl;
	
	
	// cur_psi = t265_att.z + M_PI/2;
	cur_psi = t265_att.z * 180 / M_PI + 90;

	// Compare global robot origin to first waypoint [W]
	des_psi_1 = atan2((wp_r_y[idx] - y_ic),(wp_r_x[idx] - x_ic)) * 180 / M_PI;
	err_psi_1 = des_psi_1- cur_psi;

	// Compare second waypoint to first waypoint [W]
	des_psi_2 = atan2((wp_r_y[idx + 1] - wp_r_y[idx]),(wp_r_x[idx + 1] - wp_r_x[idx])) * 180 / M_PI;
	err_psi_2 = des_psi_2- cur_psi;

	err_psi = k*err_psi_1 + (1-k)*err_psi_2;

	// d gain : - d_psi*t265_ang_vel.z;
	err_psi_dot = p_psi*err_psi ;

	arr_psi.data[0] = err_psi_dot;

	std::cout << " err_ psi1 : " << err_psi_1 << std::endl;
	std::cout << " err_ psi2 : " << err_psi_2 << std::endl;
	std::cout << " err_ psi_dot : " << err_psi_dot << std::endl;
	idx_ros.data = idx;
}

void corner_decision_Callback(const std_msgs::Bool::ConstPtr& decision)
{
	corner_flag = decision->data;
	//std::cout << "corner_flag : " << corner_flag <<std::endl;
}

