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
#include <std_msgs/Bool.h>

#define const_vel 10
#define L 0.43
#define wp_num 10

//~~~~~~~~variable
double temp1, temp2;
double c, s;
double t265_px, t265_py;

geometry_msgs::Vector3 pos;
geometry_msgs::Quaternion rot;
geometry_msgs::Vector3 t265_ang_vel;

std_msgs::Float32MultiArray wp_set_sub;
std_msgs::Float32MultiArray arr_psi;
// data for rosbag [JH & HW]
std_msgs::Float32MultiArray arr_cam_att_2;
std_msgs::Float32MultiArray arr_atan2;
std::vector<float> wp_r_x;
std::vector<float> wp_r_y;

bool corner_flag = false;
bool flag = 0;

// control variable
float des_psi;
double e_x, e_y, e_psi;
double p_x, i_x, d_x;
double p_y, i_y, d_y;
double p_psi = 1;
double i_psi = 0.3;
double d_psi = 0.1;
double cmd_psi = 0;
double distance = 1.;
double threshold = 0.01; //need to change
double freq = 30;//controller loop frequency
int idx = 0;

Eigen::Vector3d cam_att;

// Function 
// ----------------------------------------------------------------------------
void rotCallback(const geometry_msgs::Quaternion& msg);
void posCallback(const geometry_msgs::Vector3& msg);
void wp_r_x_Callback(const std_msgs::Float32MultiArray::ConstPtr& array);
void wp_r_y_Callback(const std_msgs::Float32MultiArray::ConstPtr& array);
void t265OdomCallback(const nav_msgs::Odometry::ConstPtr& msg);
void vec_delete_float(std::vector<float> &vec);
void cal_rot_wp();
void pos_ctrl();
void corner_decision_Callback(const std_msgs::Bool::ConstPtr& decision);
// ----------------------------------------------------------------------------

int main(int argc, char **argv)
{   
    ros::init(argc, argv,"magv_controller");
    ros::NodeHandle nh;  

	wp_set_sub.data.resize(wp_num*2);
	arr_psi.data.resize(1);
	// data for rosbag [JH & HW]
	arr_cam_att_2.data.resize(1);
	arr_atan2.data.resize(1);

	ros::Publisher pub_psi = nh.advertise<std_msgs::Float32MultiArray>("pub_psi", 1000);
	// data for rosbag [JH & HW]
	ros::Publisher cam_att_2_pub = nh.advertise<std_msgs::Float32MultiArray>("cam_att_2_pub", 100);
	ros::Publisher atan2_pub = nh.advertise<std_msgs::Float32MultiArray>("atan2_pub", 100);

	ros::Subscriber d435_rot=nh.subscribe("/d435_rot",100,rotCallback);
	ros::Subscriber d435_pos=nh.subscribe("/d435_pos",100,posCallback);//ros::TransportHints().tcpNoDelay());
	ros::Subscriber waypoint_r_x_sub =nh.subscribe("wp_r_x",100,wp_r_x_Callback);
    ros::Subscriber waypoint_r_y_sub =nh.subscribe("wp_r_y",100,wp_r_y_Callback);
	ros::Subscriber t265_odom=nh.subscribe("/camera/odom/sample",100,t265OdomCallback,ros::TransportHints().tcpNoDelay());
	ros::Subscriber corner_decision_sub=nh.subscribe("/corner_decision",100,corner_decision_Callback);

	ros::Rate loop_rate(100);
	
	while (ros::ok()) 
	{
		pub_psi.publish(arr_psi);
		atan2_pub.publish(arr_atan2);
		cam_att_2_pub.publish(arr_cam_att_2);
		ros::spinOnce();
		loop_rate.sleep();
	}
    return 0;
}


void posCallback(const geometry_msgs::Vector3& msg){
	pos.x=msg.x;
	pos.y=msg.y;
	pos.z=-msg.z;
	if(flag == 1) pos_ctrl();
	// ROS_INFO("Translation - [x: %f  y:%f  z:%f]",pos.x,pos.y,pos.z);
}


void wp_r_x_Callback(const std_msgs::Float32MultiArray::ConstPtr& array)
{
	vec_delete_float(wp_r_x);
	for (int i = 0; i < wp_num; i++) 
	{
		wp_r_x.push_back(array->data[i]);	
	}
	// t265_px = pos.x;
	// t265_py = pos.y;
	// cal_rot_wp();
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


void rotCallback(const geometry_msgs::Quaternion& msg)
{
	rot.x=msg.x;
	rot.y=msg.y;
	rot.z=msg.z;
	rot.w=msg.w;
}

void t265OdomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
	// t265_lin_vel=msg->twist.twist.linear;
	t265_ang_vel=msg->twist.twist.angular;
	// t265_quat=msg->pose.pose.orientation;
	tf::Quaternion quat;
	tf::quaternionMsgToTF(rot,quat);
	tf::Matrix3x3(quat).getRPY(cam_att(0),cam_att(1),cam_att(2));
}

void vec_delete_float(std::vector<float> &vec)
{
    vec.clear();
    std::vector<float>().swap(vec);
}

// void cal_rot_wp()
// {	
// 	vec_delete_double(wp_r_x);
// 	vec_delete_double(wp_r_y);
// 	c = cos(cam_att(2));
// 	s = sin(cam_att(2));
// 	for(int i = 0; i < wp_num; i++)
// 	{
// 		temp1 = c*wp_set_sub.data[2*i] - s*wp_set_sub.data[2*i+1] + t265_px;
// 		temp2 = s*wp_set_sub.data[2*i] + c*wp_set_sub.data[2*i+1] + t265_py;
// 		wp_r_x.push_back(temp1);
// 		wp_r_y.push_back(temp2);
// 		// std::cout << "wp_r_x: " << wp_r_x[i] << " wp_r_y: " << wp_r_y[i]  << std::endl;
// 	}
// }

void pos_ctrl()
{
	//if( distance < threshold) //idx += 1;

	//distance = sqrt(pow((wp_r_x[idx]-pos.x),2)+pow((wp_r_y[idx]-pos.y),2));
	des_psi = atan2((wp_r_y[1]-wp_r_y[0]),(wp_r_x[1]-wp_r_x[0]))- (rot.z+M_PI/2);
	// std::cout << " atan2 : " << atan2((wp_r_y[idx+1]-wp_r_y[idx]),(wp_r_x[idx+1]-wp_r_x[idx])) << std::endl;
	// std::cout << " cam_att(2) + M_PI/2 : " <<  (cam_att(2)+M_PI/2) <<std::endl;
	arr_cam_att_2.data[0] = rot.z;
	arr_atan2.data[0] = atan2((wp_r_y[1]-wp_r_y[0]),(wp_r_x[1]-wp_r_x[0]));
	//std::cout << " des_ psi : " << des_psi << std::endl;
	arr_psi.data[0] = des_psi;
	// R = L / tan(e_psi);
	
	// cmd_psi = p_psi*e_psi + i_psi*e_psi*(1/freq) - d_psi*t265_ang_vel.z;
	// std::cout << " t265 ang vel : " << t265_ang_vel.z << std::endl;
	// std::cout << " cmd_psi : " << cmd_psi << std::endl;

	// 0< const_vel < 255
}

void corner_decision_Callback(const std_msgs::Bool::ConstPtr& decision)
{
	corner_flag = decision->data;
	std::cout << "corner_flag : " << corner_flag <<std::endl;
}