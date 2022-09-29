#include "get_waypoint.hpp"
#include <chrono>

using namespace cv;
using namespace std;

// select line which we want to tracking(by color) - demo [HW]
int color = 1;
// color : 1 = red / color : 2 =blue

// To subscribe t265 information [W]
// ---------------------------------------------------------------------
void pos_Callback(const geometry_msgs::Vector3& msg);
void rot_Callback(const geometry_msgs::Quaternion& msg);
void idxCallback(const std_msgs::Int16& msg);
// For counting time [JH]
// ---------------------------------------------------------------------
auto k_time =std::chrono::high_resolution_clock::now();
auto start =std::chrono::high_resolution_clock::now();
auto pub_time =std::chrono::high_resolution_clock::now();

// K means function declaration [W]
Mat K_Means(Mat Input, int K);

int main(int argc, char **argv)
{
    // corner flag [JH]
    corner_flag.data = false; // straight line -> false, corner -> true
    
    // save image for each name [W]
    time_t timer;
    struct tm* t;
    
    ros::init(argc, argv, "ros_realsense_opencv_tutorial");
    ros::NodeHandle nh;

    // ROS Publisher
    ros::Publisher waypoints_r_x = nh.advertise<std_msgs::Float32MultiArray>("wp_r_x", wp_num);
    ros::Publisher waypoints_r_y = nh.advertise<std_msgs::Float32MultiArray>("wp_r_y", wp_num);
    ros::Publisher corner_decision = nh.advertise<std_msgs::Bool>("corner_decision", 5);
    ros::Publisher d435_origin_pub = nh.advertise<std_msgs::Float32MultiArray>("d435_origin", wp_num);
    
    // ROS Subscriber [W]
    ros::Subscriber d435_pos = nh.subscribe("/d435_pos",5,pos_Callback);
    ros::Subscriber d435_rot = nh.subscribe("/d435_rot",5,rot_Callback);
    ros::Subscriber test = nh.subscribe("/pub_idx",100,idxCallback);
    
    //get camera info
    // rs2::pipeline pipe;
    // rs2::config cfg;
    // rs2::frameset frames;
    // rs2::frame color_frame;   
    // cfg.enable_stream(RS2_STREAM_COLOR, 1280,720, RS2_FORMAT_BGR8, 30);
    // pipe.start(cfg);    

    // perspective transform [JH]
    src_p[0] = Point2f(443,478);
    src_p[1] = Point2f(336,720);
    src_p[2] = Point2f(855,474);
    src_p[3] = Point2f(940,720);

    dst_p[0] = Point2f(604-268/hf,720-268/hf*3/2);
    dst_p[1] = Point2f(604-268/hf,720);
    dst_p[2] = Point2f(604+268/hf,720-268/hf*3/2);
    dst_p[3] = Point2f(604+268/hf,720);
    
    // if it doesn't exist, it cause error [JH]
    // for(int i=0; i < 50; i ++)
    // {
    //     frames = pipe.wait_for_frames();
    // }

    while(ros::ok())
    {   
        // for counting time [JH]
        timer = time(NULL);
        t = localtime(&timer);
        start = std::chrono::high_resolution_clock::now();
        
        // get image from d435 [JH]
        // frames = pipe.wait_for_frames();
        // color_frame = frames.get_color_frame();

        // To avoid core dumped [W]
        set_array(wp_r_x, wp_num);
        set_array(wp_r_y, wp_num); 
        set_array(d435_origin, 2);

        // to calculate global waipoint's coordinate [W]
        x_ic = pos.x;
        y_ic = pos.y;
        d435_origin.data[0] = x_ic;
        d435_origin.data[1] = y_ic;
        cos_ic = cos(t265_att.z);
        sin_ic = sin(t265_att.z);

        Mat src = imread("/home/mrl/.ros/sample_img_29.57.png", 1);      
        // Mat src(Size(1280,720), CV_8UC3, (void*)color_frame.get_data(), Mat::AUTO_STEP);
        
        Mat perspective_mat = getPerspectiveTransform(src_p, dst_p);

        // perspective matrix [W]
        warpPerspective(src, dst, perspective_mat, Size(1280,720));
        imwrite("bev.png",dst); 

        // read image [HW]
        // Mat dst = imread("/home/mrl/.ros/sample_img_28.3.png", 1);

        // K_Means function [JH]
        int Clusters = 9;
        // Mat res = K_Means(dst, Clusters);
        Mat res = dst;   
        // imwrite("bev2.png",res);      

        // counting time for kmeans [JH]
        k_time=std::chrono::high_resolution_clock::now();
        
        // For remove BEV edge [HW]
        line(res, Point(0,145),Point(411,720),Scalar(0,0,0), 1); 
        line(res, Point(1228,0),Point(790,720),Scalar(0,0,0), 1);
        
        // BGR -> HSV [HW]
        cvtColor(res, hsv, COLOR_BGR2HSV);

        // ========================================================================         
        // HSV detect [W]
        // mask : color 1 is red / color 2 is blue - demo version [W]
        if(color ==1) // red
        {
          inRange(hsv, lower_red, upper_red, mask);  
          inRange(hsv, lower_red_sc, upper_red_sc, mask_sc);
          bitwise_or(mask, mask_sc, mask);
          bitwise_and(res, res, image, mask);
        }
        else if (color ==2) // blue
        {
          inRange(hsv, lower_blue, upper_blue, mask);  
          bitwise_and(res, res, image, mask);
        }
        // ========================================================================


        // Image Morpology [W]
        // Mat mask = imread("/home/mrl/.ros/mask_29.6.png", 0);
	    // erode(mask, img_erode, Mat::ones(Size(3,3),CV_8UC1),Point(-1,-1),3);
        // dilate(img_erode, img_dilate, Mat::ones(Size(3, 3), CV_8UC1), Point(-1, -1), 3);
        // std::cout << src.channels() <<std::endl;
        // imwrite("after_erode.png",img_erode);
        // imwrite("after_dilate.png",img_dilate);
        // mask = img_dilate;

        // find contours [HW]
        // line(mask, Point(0,300),Point(300,720),Scalar(0), 1); 
        findContours(mask, contours, hierachy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE); 
        drawContours(image, contours, -1, Scalar(255, 0, 0), 5);
        // imwrite("contour.png", image);  

        // get contour's y_val( every y ), x_val (every x ) [JH]
        for ( int i = 0; i < contours.size(); i++)
        {
            for ( int j = 0; j < contours[i].size(); j++)
            {
                y_val.push_back(contours[i][j].y);
            }
        }
        for ( int i = 0; i < contours.size(); i++)
        {
            for ( int j = 0; j < contours[i].size(); j++)
            {
                x_val.push_back(contours[i][j].x);
            }
        }
    
        // contour's x,y min/max value [JH]
        float min = *min_element(y_val.begin(),y_val.end());
        float max = *max_element(y_val.begin(),y_val.end());
        float x_left = *min_element(x_val.begin(),x_val.end());
        float x_right = *max_element(x_val.begin(),x_val.end());
        
        // contour's bottom x value [JH]
        for ( int i = 0; i < contours.size(); i++)
        {
            for ( int j = 0; j < contours[i].size(); j++)
            {
                if(contours[i][j].y == max)
                {
                    bottom_x.push_back(contours[i][j].x);
                }
            }
        }

        // corner [JH]
        // get avg of contour's bottom x value [JH]
        float sum_bottom_x = accumulate(bottom_x.begin(),bottom_x.end(),0);
        float mean_bottom_x = sum_bottom_x/bottom_x.size();
        
        // if system detect more than 2 contours -> connect every contours [JH]
        for (int i=0; i < contours.size(); i++)
        {
            for (int j=0; j < contours[i].size() ; j++)
            {
                contours_sum.push_back(contours[i][j]);
            }
        }

        // fitLine() function to detect representive line [W]
        fitLine(contours_sum, detected_line, CV_DIST_L2, 0, 0.01, 0.01);
        vx = detected_line[0]; // fitline vector x
        vy = detected_line[1]; // fitline vector y
        gradient = vy/vx; //gradient of fitline
        x = float(detected_line[2]); // point of fitline
        y = float(detected_line[3]); // point of fitline
    
        // get (x,y) of detected line in converted coordinate [JH]
        converted_x = convert_x(x);
        converted_y = convert_y(y);
        top_y = convert_y(min);
        left_x = convert_x(x_left);
        right_x = convert_x(x_right);
    
        // get the source of drawing straight line [JH]
        upper_x = float(-vx/vy*(top_y-converted_y)+converted_x);
        lower_x = float(-vx/vy*(-1*converted_y) + converted_x);

        // visualization representive line [W]
        line(res, Point(inv_convert_x(upper_x),inv_convert_y(top_y)), Point(inv_convert_x(lower_x),inv_convert_y(0)),Scalar(0,0,255), 3);
        line(mask_line, Point(inv_convert_x(upper_x),inv_convert_y(top_y)), Point(inv_convert_x(lower_x),inv_convert_y(0)),Scalar(255), 3);

        // Corner detection Algorithm v_1 [JH]
        // if ( abs(gradient) <= 1.5 ) corner_flag.data = true;
        // else if ( abs(gradient) > 1.5 ) corner_flag.data = false;
        // std::cout << "gradient :  " << gradient << "    corner flag : "<< corner_flag.data << std::endl;  

        // Corner detection Algorithm v_2 [W]
        bitwise_or(mask, mask_line, mask_combination);

        for (int y = 0; y < mask.rows; y++)
		    for (int x = 0; x < mask.cols; x++)
		    {
			    if(mask.at<int>(y,x)==255)
                mask_count ++;
			}
        
        for (int y = 0; y < mask.rows; y++)
		    for (int x = 0; x < mask.cols; x++)
		    {
			    if(mask_combination.at<int>(y,x)==255)
                line_count ++;
			}
        // std::cout << "a : "<<mask_count << std::endl;
        // std::cout << "b : "<<line_count << std::endl;

        if ((line_count - mask_count) >= 200 & fabs(gradient) <= 1.5) corner_flag.data = true;
        else if (idx>=8)  corner_flag.data = false;
        
        // if our's objective line is straight [W]
        if (corner_flag.data == false) // straight 
        {
            for(int i=0; i<wp_num; i++)
            {
                wp_y.push_back(top_y/wp_num*(i+1));
                wp_x.push_back((-vx/vy*(wp_y[i]-converted_y)+converted_x));
                wp_r_x.data[i] =  x_ic + cos_ic * wp_x[i]*dist_pix  - sin_ic * wp_y[i]*dist_pix; //* distance_of_pixel
                wp_r_y.data[i] =  y_ic + sin_ic * wp_x[i]*dist_pix  + cos_ic * wp_y[i]*dist_pix;
            }
        }
        
        // if our's objective lines is radius [W]
        else if(corner_flag.data == true) // rotation
        {   
            for(int i=0; i<wp_num; i++) // circle waypoint y
            {
                float theta = i *wp_num *(PI / 180);

                wp_y.push_back(top_y * sin(theta));
            }
            if (vy/vx > 0) // turn left - waypoint x
            {
                for(int i=0; i<wp_num; i++)
                {
                    float theta = i * wp_num * (PI / 180); //9/10 
                    wp_x.push_back(convert_x(mean_bottom_x - (top_y - top_y*cos(theta))));
                }
            } 
            else if (vy/vx < 0) // turn right - waypoint x
            {
                for(int i=0; i<wp_num; i++)
                {
                    float theta = i*wp_num * (PI / 180);
                    wp_x.push_back(convert_x(mean_bottom_x + (top_y - top_y*cos(theta))));
                }
            }
            for(int i=0; i<wp_num; i++)
            {
                wp_r_x.data[i] =  x_ic + cos_ic * wp_x[i]*dist_pix  - sin_ic * wp_y[i]*dist_pix;// * distance_of_pixel;
                wp_r_y.data[i] =  y_ic + sin_ic * wp_x[i]*dist_pix  + cos_ic * wp_y[i]*dist_pix;// * distance_of_pixel;
            }
        }
        // visualization waypoint
        for(int i=0; i<wp_num; i++)
        {
            circle(res, Point(inv_convert_x(wp_x[i]), inv_convert_y(wp_y[i])), 5, Scalar(255,255,255), 3);
        }

        // Publish topics [W]   
        corner_decision.publish(corner_flag);
        waypoints_r_x.publish(wp_r_x);
        waypoints_r_y.publish(wp_r_y);
        d435_origin_pub.publish(d435_origin);

        pub_time=std::chrono::high_resolution_clock::now();
        
        std:: cout << "k_means: " << chrono::duration_cast<chrono::milliseconds>(k_time - start).count() <<"ms"<< std::endl;
        
        // vector initialization [W]
        vec_delete(x_val);
        vec_delete(y_val);
        vec_delete_float(wp_x);
        vec_delete_float(wp_y);
        vec_delete(bottom_x);
        vec_delete_p(contours_sum);

        mask_count = 0;
        line_count = 0;

        // naming by time [JH]
        char filename_sample_img[200];
        sprintf(filename_sample_img, "sample_img_%d.%d.png", t->tm_min, t->tm_sec);

        char filename[200];
        sprintf(filename, "res_%d.%d.png", t->tm_min, t->tm_sec);

        char filename_mask[200];
        sprintf(filename_mask, "mask_%d.%d.png", t->tm_min, t->tm_sec);

        // save image name depends on time [JH]
        imwrite(filename_sample_img,src); 
        
        
        // imwrite(filename,res);
        // imwrite(filename_mask,mask);

        // save image [JH]
        imwrite("res.png", res);     
        imwrite("mask.png", mask);     
        // imwrite("mask_line.png", mask_line); 
        // imwrite("mask_com.png", mask_combination); 
        
        ros::spinOnce();        
    }
    return 0;
}

void pos_Callback(const geometry_msgs::Vector3& msg){
	pos.x=msg.x;
	pos.y=msg.y;
	pos.z=-msg.z;
}


void rot_Callback(const geometry_msgs::Quaternion& msg)
{
	rot.x=msg.x;
	rot.y=msg.y;
	rot.z=msg.z;
	rot.w=msg.w;

    tf::Quaternion quat;
	tf::quaternionMsgToTF(rot,quat);
	tf::Matrix3x3(quat).getRPY(t265_att.x,t265_att.y,t265_att.z);	
}
void idxCallback(const std_msgs::Int16& msg)
{
    idx = msg.data;
}

Mat K_Means(Mat Input, int K) {
	Mat samples(Input.rows * Input.cols, Input.channels(), CV_32F);
	for (int y = 0; y < Input.rows; y++)
		for (int x = 0; x < Input.cols; x++)
			for (int z = 0; z < Input.channels(); z++)
				if (Input.channels() == 3) {
					samples.at<float>(y + x * Input.rows, z) = Input.at<Vec3b>(y, x)[z];
				}
				else {
					samples.at<float>(y + x * Input.rows, z) = Input.at<uchar>(y, x);
				}

	Mat labels;
	int attempts = 5;
	Mat centers;
    // Need to modify function arguement [W]
	kmeans(samples, K, labels, TermCriteria(CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 10, 0.1), attempts, KMEANS_PP_CENTERS, centers);

	Mat new_image(Input.size(), Input.type());
	for (int y = 0; y < Input.rows; y++)
		for (int x = 0; x < Input.cols; x++)
		{
			int cluster_idx = labels.at<int>(y + x * Input.rows, 0);
			if (Input.channels()==3) {
				for (int i = 0; i < Input.channels(); i++) {
					new_image.at<Vec3b>(y, x)[i] = centers.at<float>(cluster_idx, i);
				}
			}
			else {
				new_image.at<uchar>(y, x) = centers.at<float>(cluster_idx, 0);
			}
		}
	return new_image;
}