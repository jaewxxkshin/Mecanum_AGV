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

#define wp_num 10
#define max_dist 0.3
#define max_k 1.0

std_msgs::Float32MultiArray wp_set_sub;
std_msgs::Float32MultiArray arr_psi;
std_msgs::Int16 idx_ros;
std_msgs::Int16 g_flag_ros;
std_msgs::Float32 dist_ros;
std_msgs::Float32 k_ros;

geometry_msgs::Vector3 pos;
geometry_msgs::Quaternion rot;
geometry_msgs::Vector3 t265_ang_vel;
geometry_msgs::Vector3 t265_att;

std::vector<float> wp_r_x;
std::vector<float> wp_r_y;

bool corner_flag = false;
bool flag = 0;
bool d_flag = true;
bool finish_flag = false;
//---------------------------[W]
bool straight_mode = 1;
//bool radius_mode = 0;
//---------------------------

// control variable
double p_psi = 5.;
// double d_psi = 0.;
double cmd_psi = 0;
double distance = 0.;
double ver_a = 0.;
double ver_b = 0.;
double ver_c = 0.;
double ver_dist = 0.;
double theta_robot = 0.;
double theta_line = 0.;
double freq = 30;//controller loop frequency
float err_psi = 0;
float err_psi_1 = 0;
float err_psi_2 = 0;
float err_psi_dot = 0;
float des_psi_1 = 0;
float des_psi_2 = 0;
float prev_psi = 0;
float cur_psi = 0;
float k = 0;
float x_ic, y_ic = 0;
// Eigen::Vector3d cam_att;

// jh 
double temp_y1 = 0.0;
double temp_y2 = 0.0;
double temp_x1 = 0.0;
double temp_x2 = 0.0;
int g_flag = 0;
bool prev_corner = false;
bool idx_flag = false;
int16_t idx = 0;
int16_t idx_2 = 0;

// variable for dp_product
float gradient =0.0;
float dp_val =0.0;
std::vector<float> vector1;
std::vector<float> vector2;

// Function 
// ----------------------------------------------------------------------------
void rotCallback(const geometry_msgs::Quaternion& msg);
void posCallback(const geometry_msgs::Vector3& msg);
// void t265OdomCallback(const nav_msgs::Odometry::ConstPtr& msg);
void wp_r_x_Callback(const std_msgs::Float32MultiArray::ConstPtr& array);
void wp_r_y_Callback(const std_msgs::Float32MultiArray::ConstPtr& array);
void originCallback(const std_msgs::Float32MultiArray::ConstPtr& array);
void vec_delete_float(std::vector<float> &vec);
void cal_rot_wp();
void yaw_ctrl();
void corner_decision_Callback(const std_msgs::Bool::ConstPtr& decision);
void finish_decision_Callback(const std_msgs::Bool::ConstPtr& decision);
// ----------------------------------------------------------------------------


int main(int argc, char **argv)
{   
    ros::init(argc, argv,"magv_controller");
    ros::NodeHandle nh;  

	wp_set_sub.data.resize(wp_num*2);
	arr_psi.data.resize(5);
	
	ros::Publisher pub_psi = nh.advertise<std_msgs::Float32MultiArray>("pub_psi", 1000);
	ros::Publisher pub_idx = nh.advertise<std_msgs::Int16>("pub_idx", 1000);
	ros::Publisher pub_g_flag = nh.advertise<std_msgs::Int16>("g_flag", 1000);
	ros::Publisher pub_dist = nh.advertise<std_msgs::Float32>("pub_dist", 1000);

	ros::Publisher pub_k = nh.advertise<std_msgs::Float32>("pub_k", 1000);
	
    
	ros::Subscriber d435_rot=nh.subscribe("/d435_rot",100,rotCallback);
	ros::Subscriber d435_pos=nh.subscribe("/d435_pos",100,posCallback);
	ros::Subscriber waypoint_r_x_sub =nh.subscribe("wp_r_x",100,wp_r_x_Callback);
    ros::Subscriber waypoint_r_y_sub =nh.subscribe("wp_r_y",100,wp_r_y_Callback);
	ros::Subscriber corner_decision_sub=nh.subscribe("/corner_decision",100,corner_decision_Callback);
	ros::Subscriber finish_decision_sub=nh.subscribe("/finish_decision",100,finish_decision_Callback);
	ros::Subscriber d435_origin_sub =nh.subscribe("/d435_origin",100,originCallback);
	// ros::Subscriber t265_odom=nh.subscribe("/camera/odom/sample",100,t265OdomCallback,ros::TransportHints().tcpNoDelay());
	
	ros::Rate loop_rate(100);
	
	while (ros::ok()) 
	{
		pub_psi.publish(arr_psi);
		pub_idx.publish(idx_ros);
		pub_g_flag.publish(g_flag_ros);
		pub_dist.publish(dist_ros);
		pub_k.publish(k_ros);
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
	if (corner_flag == false && finish_flag==false)	d_flag = true; // straight line -> d_flag = true
	
	if (d_flag == true)
	{
		idx_flag = false; 
		// wp update ====================
		vec_delete_float(wp_r_x);
		for (int i = 0; i < wp_num; i++) 
		{
		wp_r_x.push_back(array->data[i]);	
		}
		// ==============================
		idx = 0;
		prev_corner == false; // straight 
	}
	// straight line -> corner 
	if ( prev_corner == false && corner_flag == true ) idx_flag = true; // straight line -> corner : idx_flag = true
	// it means when the robot detects corner, idx_flag+idx -> g_flag ( for x->y )
	if (idx_flag == true && idx >4)	g_flag ++;
	g_flag_ros.data = g_flag;

	if (corner_flag == true) 
	{
		d_flag = false; // don't update wp 
		prev_corner == true;
	}

	if(finish_flag == true) d_flag= false; // finish flag = true -> don't update wp

	// std::cout << "corner flag : \t" << corner_flag << "\td_flag : \t" << d_flag << std::endl;
	
	flag = 1;
	// idx will initialized when new waypoint is created
}

void wp_r_y_Callback(const std_msgs::Float32MultiArray::ConstPtr& array)
{
	if (corner_flag == false) d_flag = true; // straight line -> d_flag = true
	if (d_flag == true)
	{
		// wp update ====================		
		vec_delete_float(wp_r_y);
		for (int i = 0; i < wp_num; i++) 
		{
			wp_r_y.push_back(array->data[i]);	
		}
		// ==============================
	}
	if (corner_flag == true) d_flag = false; // corner -> d_flag = false : don't update wp
}

void originCallback(const std_msgs::Float32MultiArray::ConstPtr& array)
{
	// x_ic = array->data[0];
	// y_ic = array->data[1];
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

// void t265OdomCallback(const nav_msgs::Odometry::ConstPtr& msg){
// 	// t265_lin_vel=msg->twist.twist.linear;
// 	t265_ang_vel=msg->twist.twist.angular;
// 	// t265_quat=msg->pose.pose.orientation;
// 	// tf::Quaternion quat;
// 	// tf::quaternionMsgToTF(rot,quat);
// 	// tf::Matrix3x3(quat).getRPY(cam_att(0),cam_att(1),cam_att(2));
// 	// std::cout << " =====created camera att 2 : " << cam_att(2) << std::endl;
// }

void vec_delete_float(std::vector<float> &vec)
{
    vec.clear();
    std::vector<float>().swap(vec);
}

void yaw_ctrl()
{	
	
	// update index by using dot product =========================================
	// //vector 1 : robot - wp[idx]
	vec_delete_float(vector1);
	vector1.push_back(wp_r_x[idx]-pos.x);
	vector1.push_back(wp_r_y[idx]-pos.y);

	//vector 2 : wp[idx] - wp[idx+1]
	vec_delete_float(vector2);
	if(idx==9){
		vector2.push_back(wp_r_x[idx]-wp_r_x[idx-1]);
		vector2.push_back(wp_r_y[idx]-wp_r_y[idx-1]);
	}
	else{
		vector2.push_back(wp_r_x[idx+1]-wp_r_x[idx]);
		vector2.push_back(wp_r_y[idx+1]-wp_r_y[idx]);
	}
	// gradient = (wp_r_y[idx+1]-wp_r_y[idx])/(wp_r_x[idx+1]-wp_r_x[idx]);
	// vector2.push_back(1/(sqrt(1+pow(gradient,2))));
	// vector2.push_back(gradient/(sqrt(1+pow(gradient,2))));
	
	// Dot_ Product
	dp_val = (vector1[0] * vector2[0]) + (vector1[1] * vector2[1]);
	
	if( dp_val<0)idx++;
	//==============================================
	// std::cout << "index : " << idx <<std::endl;
	// std::cout << "vector1_x : " << vector1[0] << "\tvector1_y : " << vector1[1] << "\tvector2_x : " << vector2[0] << "\tvector2_y : " << vector2[1] << "\tDP_theta : "<< dp_val << std::endl;
	// std::cout << "dp_val: " << dp_val <<std::endl; 
	//============================================================================

	// if ( g_flag%2 == 0 )
	// {
 	// 	gradient = -(wp_r_x[idx+1]-wp_r_x[idx])/(wp_r_y[idx+1]-wp_r_y[idx]);
	// 	// f1(x)
	// 	// double tmp_x = pos.x;
	// 	// double tmp_y = pos.y;
	// 	temp_y1 = gradient *(pos.x - wp_r_x[idx])+ wp_r_y[idx];
	// 	// f2(x)
	// 	// temp_y2 = gradient *(pos.x - wp_r_x[idx+1])+ wp_r_y[idx+1];
	// 	if (wp_r_y[1] - wp_r_y[0]>0 && pos.y > temp_y1 ) idx++; // y++
	// 	if (wp_r_y[1] - wp_r_y[0]<0 && pos.y < temp_y1 ) idx++; // y--
	// }

	// if ( g_flag%2 == 1 )
	// {
	// 	// x
	// 	// gradient = -(wp_r_x[idx+1]-wp_r_x[idx])/(wp_r_y[idx+1]-wp_r_y[idx]);
	// 	gradient = -(wp_r_y[idx+1]-wp_r_y[idx])/(wp_r_x[idx+1]-wp_r_x[idx]);		
	// 	// f1(x)
	// 	// double tmp_x = pos.x;
	// 	// double tmp_y = pos.y;
	// 	temp_x1 = gradient *(pos.y - wp_r_y[idx])+ wp_r_x[idx];
	// 	// f2(x)
	// 	temp_x2 = gradient *(pos.y - wp_r_y[idx+1])+ wp_r_x[idx+1];
	// 	if (wp_r_x[1] - wp_r_x[0]>0 && pos.x > temp_x1 ) idx++; 
	// 	if (wp_r_x[1] - wp_r_x[0]<0 && pos.x < temp_x1 ) idx++;
	// }

	cur_psi = t265_att.z * 180 / M_PI + 90;

	if ( idx < wp_num-1 )
	{
		// Compare global robot origin to first waypoint [W]
		// err_psi_1 = atan2((wp_r_y[idx] - pos.y), (wp_r_x[idx] - pos.x)) * 180 / M_PI;
		// // std::cout << "err_psi_1_before_if : " << err_psi_1 << std::endl;
		// if (err_psi_1 < 0) err_psi_1 += 90;
		// else if (err_psi_1 >= 0) err_psi_1 -= 90; 
		// std::cout << "err_psi_1_before : " << err_psi_1 << std::endl;
		
		// JH idea 
		des_psi_1 = atan2((wp_r_y[idx] - pos.y), (wp_r_x[idx] - pos.x)) * 180 / M_PI;
		if(des_psi_1 <= -cur_psi) des_psi_1 += 360 ;
		err_psi_1 = des_psi_1 - cur_psi;
		
		// std::cout << "err_psi_1_after____ : " << err_psi_1 << std::endl;
		// Compare second waypoint to first waypoint [W]
		des_psi_2 = atan2((wp_r_y[idx + 1] - wp_r_y[idx]),(wp_r_x[idx + 1] - wp_r_x[idx])) * 180 / M_PI; 
		if (des_psi_2 < cur_psi-180)	des_psi_2 += 360.0;
		err_psi_2 = des_psi_2- cur_psi;
		// vir d [JH]=======================================================================
		// theta_robot = fabs(atan2 (wp_r_y[idx]-pos.y,wp_r_x[idx]-pos.x));

		// if(idx==0) idx=1;
		
		// theta_line = atan2(wp_r_y[idx]-wp_r_y[idx-1],wp_r_x[idx]-wp_r_x[idx-1])-M_PI/2;
		
		// theta_robot = fabs(theta_robot - theta_line);
		// // theta_robot = theta_robot - t265_att.z;

		// if(fabs(theta_robot)>M_PI/2) theta_robot = M_PI - theta_robot; 

		// theta_robot = fabs(theta_robot);
		// distance = sqrt( pow((wp_r_x[idx]-pos.x),2) + pow((wp_r_y[idx]-pos.y),2) );	
		// ver_d = distance * cos(theta_robot);
		
		//==============================================================================

		//vertical distance[HW]=========================================================================
		idx_2 = idx;
		if(idx_2==0) idx_2 = 1;
		// ax + by + c = 0     a -> ver_a, b -> ver_b, c -> ver_c
		ver_a = wp_r_y[idx_2] - wp_r_y[idx_2-1];
		ver_b = wp_r_x[idx_2-1] - wp_r_x[idx_2];
		ver_c = -wp_r_x[idx_2-1] * (wp_r_y[idx_2]-wp_r_y[idx_2-1]) + wp_r_y[idx_2-1] * (wp_r_x[idx_2] - wp_r_x[idx_2-1]);
		// distance between point and line[HW]
		ver_dist = fabs(ver_a * pos.x + ver_b * pos.y + ver_c) / sqrt(pow(ver_a,2) + pow(ver_b,2));

		//==========================================================================================
		// std::cout << "pos.x : " << pos.x << "\tpos.y : " << pos.y << std::endl;
		// distance = sqrt( pow((wp_r_x[idx]-pos.x),2) + pow((wp_r_y[idx]-pos.y),2) );	
		// if ( distance > max_dist ) distance = max_dist;
		// k = distance * ( max_k / max_dist );
		// if(fabs(k)>1) k = k/fabs(k);
		if ( ver_dist > max_dist ) ver_dist = max_dist;
		dist_ros.data = ver_dist;
		k = ver_dist * (max_k / max_dist);
		if(fabs(k)>1) k = k/fabs(k);

		k_ros.data = k;

		// 0.1 -> ++0.1
		// if (corner_flag == false) 
		err_psi = k * err_psi_1 + (1-k) * err_psi_2;
		err_psi_dot = p_psi * err_psi; //- d_psi * t265_ang_vel.z;

	}
	else if ( idx == wp_num-1 )
	{
		des_psi_2 = atan2((wp_r_y[idx] - wp_r_y[idx-1]),(wp_r_x[idx] - wp_r_x[idx-1])) * 180 / M_PI;
		if (des_psi_2 < 0)
		{
			des_psi_2 += 360.0;
		}
		err_psi_2 = des_psi_2- cur_psi;
		
		err_psi = err_psi_2;
		err_psi_dot = 2 * err_psi; // - d_psi*t265_ang_vel.z;
	}

	// d gain : - d_psi*t265_ang_vel.z;



	arr_psi.data[0] = err_psi_1;
	arr_psi.data[1] = err_psi_2;
	arr_psi.data[2] = err_psi;
	arr_psi.data[3] = cur_psi;
	arr_psi.data[4] = err_psi_dot;
	

	idx_ros.data = idx;
}

void corner_decision_Callback(const std_msgs::Bool::ConstPtr& decision)
{
	corner_flag = decision->data;
}

void finish_decision_Callback(const std_msgs::Bool::ConstPtr& decision)
{
	finish_flag = decision->data;
}







