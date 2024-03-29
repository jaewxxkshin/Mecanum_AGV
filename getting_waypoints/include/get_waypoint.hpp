#include <ros/ros.h>
#include <librealsense2/rs.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <iostream>
#include <vector>
#include <math.h>
#include <numeric>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>
#include <time.h>
#include <unistd.h> // for sleep [JH]
#include <std_msgs/Bool.h>
#include <std_msgs/Int16.h>

// version_2 [W]
// ---------------------------------------------
#include "tf/transform_datatypes.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_msgs/TFMessage.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Transform.h>

#include "nav_msgs/Odometry.h"
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

// ---------------------------------------------

// for test to publish way points [W]
std_msgs::Float32MultiArray wp_r_x;
std_msgs::Float32MultiArray wp_r_y;
std_msgs::Float32MultiArray ele_x;
std_msgs::Float32MultiArray ele_y;
std_msgs::Float32MultiArray d435_origin;
std_msgs::Bool corner_flag;
std_msgs::Bool finish_flag;

// t265_pos & att [W]
geometry_msgs::Vector3 pos;
geometry_msgs::Quaternion rot;
geometry_msgs::Vector3 t265_att;

// constant value[JH]
//if hf=2 -> 314,884 hf=3 -> 412,789
#define left_width 412
#define right_width 789
#define img_height 720
// #define distance_of_pixel 0.00132  //m 
#define PI 3.141592
#define corner_threshold 700
#define wp_num 10
#define hf 3

//=====================[W]
float dist_pix = 0.00132;
//=====================

// trying to solve <node die problem>
float minn =.0; 
float maxx =.0;
float x_left =.0; 
float x_right =.0;
float sum_bottom_x =.0;
float mean_bottom_x =.0;

// variations for corner dection algorithm [W]
int mask_count = 0;
int line_count = 0;

// variable for indexing update
int idx = 0;
bool img_flag = true; 

// Image renewal flag variation [W]
int countt = 0;

// varaitons of kmeans algorithm
int width, height, n, nPoints, cIndex, iTemp;
float upper_x, lower_x, top_y, converted_x, converted_y, right_x, left_x,x, y;

// const int cluster_k = 8;

// variation after applying fitLine() function [W]
float vx, vy;
// x, y positions [W]
float x_pos, y_pos;
// for fitline gradient [JH]
float gradient;

float x_ic = 0.;
float y_ic = 0.;
float cos_ic = 0.;
float sin_ic = 0.;

using namespace cv;
using namespace std;

// vector element initialization [W]
void vec_delete(vector<int> &vec)
{
    vec.clear();
    vector<int>().swap(vec);
}
void vec_delete_float(vector<float> &vec)
{
    vec.clear();
    vector<float>().swap(vec);
}

void vec_delete_p(vector<Point> &vec)
{
    vec.clear();
    vector<Point>().swap(vec);
}

void vec_delete_pair(vector<pair<int, int>> &vec)
{
    vec.clear();
    vector<pair<int, int>>().swap(vec);
}
// function for coordinate conversion [JH]
// -------------------------------------------
// int convert_x (int &x)
// { 
//   int converted_x = 0;
//   converted_x= x-((right_width-left_width)/2 + left_width);
//   return converted_x;
// }
float convert_x (const float &x)
{ 
  float converted_x = 0;
  converted_x= x-((right_width-left_width)/2.0 + left_width);
  return converted_x;
}

// int convert_y (int &y)
// {
//   int converted_y = 0;
//   converted_y= -(y-img_height);
//   return converted_y;
// }

float convert_y (const float &y)
{
  float converted_y = 0;
  converted_y= -(y-img_height);
  return converted_y;
}

int inv_convert_x( const float &x)
{
  int inv_converted_x=0;
  inv_converted_x = x+ ((right_width-left_width)/2.0 + left_width);
  return inv_converted_x;
}

int inv_convert_y( const float &y)
{
  float inv_converted_y=0;
  inv_converted_y = -y+img_height;
  return inv_converted_y;
}
// -------------------------------------------

// resize function [HW]
void set_array(std_msgs::Float32MultiArray &arr, const int &n)
{
  arr.data.resize(n);
  return;
}

// Matrix for img_proc [W]
Mat src, dst, image, mask, mask_sc, res, hsv, points, labels, centers;
Mat img_erode;
Mat img_dilate;
Mat mask_line = Mat::zeros(720, 1280, CV_8UC1);
Mat mask_combination;
        

// point for warp
Point2f src_p[4], dst_p[4];

// vector for finding contour [W]
vector<Vec4i> linesP, hierachy;
vector<vector<Point>> contours; 

// if system detect more than 2 contours -> connect every contours [JH]
vector<Point> contours_sum;

// vector for fitLine() function[W]
Vec4f detected_line;
// vector that contains all of contours's y value [W]
vector<int> y_val,  x_val;

// vector that contains waypoint's position [W]
vector<float> wp_y;
vector<float> wp_x;
vector<int> bottom_x;

// -------------------------------------------
// Red_HSV_range (based on dataset)
Scalar lower_red = Scalar(160, 90, 100);
Scalar upper_red = Scalar(180, 255, 255);
// Red_HSV_range (hsv of red - special case)
Scalar lower_red_sc = Scalar(0, 100, 100);
Scalar upper_red_sc = Scalar(20, 255, 255);
// Blue_HSV_range (based on dataset_ 1)
Scalar lower_blue = Scalar(90, 100, 100);
Scalar upper_blue = Scalar(130, 255, 255);
// -------------------------------------------


