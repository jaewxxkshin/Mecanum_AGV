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
#include <std_msgs/Float32MultiArray.h>
#include <time.h>
#include <unistd.h> // for sleep [JH]

// for test to publish way points [W]
// std_msgs::Float32MultiArray test_x;
// std_msgs::Float32MultiArray test_y;

std_msgs::Float32MultiArray wp_set;

// constant value[JH]
#define left_width 310
#define right_width 885
#define img_height 720
#define distance_of_pixel 0.00075  //m
#define pixels 72
#define PI 3.141592
#define corner_threshold 700

#define wp_num 10

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
int convert_x (int &x)
{ 
  int converted_x = 0;
  converted_x= x-((right_width-left_width)/2 + left_width);
  return converted_x;
}

int convert_y (int &y)
{
  int converted_y = 0;
  converted_y= -(y-img_height);
  return converted_y;
}

int inv_convert_x( const int &x)
{
  int inv_converted_x=0;
  inv_converted_x = x+ ((right_width-left_width)/2 + left_width);
  return inv_converted_x;
}

int inv_convert_y( const int &y)
{
  int inv_converted_y=0;
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

// point for warp
Point2f src_p[4], dst_p[4];


const int cluster_k = 4;

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

// variation after applying fitLine() function [W]
float vx, vy;

// varaitons of kmeans algorithm
int width, height,  x, y, n, nPoints, cIndex, iTemp;
int upper_x, lower_x, top_y, converted_x, converted_y, right_x, left_x;

// x, y positions [W]
float x_pos, y_pos;

// -------------------------------------------
// Red_HSV_range (based on dataset)
Scalar lower_red = Scalar(160, 100, 100);
Scalar upper_red = Scalar(180, 255, 255);
// Red_HSV_range (hsv of red - special case)
Scalar lower_red_sc = Scalar(0, 100, 100);
Scalar upper_red_sc = Scalar(20, 255, 255);
// Blue_HSV_range (based on dataset_ 1)
Scalar lower_blue = Scalar(90, 100, 100);
Scalar upper_blue = Scalar(130, 255, 255);
// -------------------------------------------

// Image renewal flag variation [W]
int countt = 0;

