
#include "get_waypoint.hpp"

using namespace cv;
using namespace std;



// select line which we want to tracking(by color) - demo [HW]
int color = 2;
// clock_t prev_time, next_time; 
// double how_long;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ros_realsense_opencv_tutorial");
    ros::NodeHandle nh;
    // ros::Publisher waypoints_x = nh.advertise<std_msgs::Float32MultiArray>("waypoints_x", 10);
    // ros::Publisher waypoints_y = nh.advertise<std_msgs::Float32MultiArray>("waypoints_y", 10);
    ros::Publisher waypoints_set = nh.advertise<std_msgs::Float32MultiArray>("waypoints_set", wp_num);
    ros::Time timer;      
    // get camera info
    rs2::pipeline pipe;
    rs2::config cfg;
    rs2::frameset frames;
    rs2::frame color_frame;   
    cfg.enable_stream(RS2_STREAM_COLOR, 1280,720, RS2_FORMAT_BGR8, 30);
    pipe.start(cfg);    

    // perspective transform [JH]
    src_p[0] = Point2f(443,478);
    src_p[1] = Point2f(336,720);
    src_p[2] = Point2f(855,474);
    src_p[3] = Point2f(940,720);

    dst_p[0] = Point2f(604-268/2,720-268/2*3/2);
    dst_p[1] = Point2f(604-268/2,720);
    dst_p[2] = Point2f(604+268/2,720-268/2*3/2);
    dst_p[3] = Point2f(604+268/2,720);
    
    // if it doesn't exist, it cause error [JH]
    for(int i=0; i < 50; i ++)
    {
        frames = pipe.wait_for_frames();
    }

    while(ros::ok())
    {   
        timer = ros::Time::now();
        // cout << timer << endl;
        // prev_time=clock();
        frames = pipe.wait_for_frames();
        color_frame = frames.get_color_frame();
        
        // cout << "count : " << countt << endl;
        
        // Image generation variation[W]
        Mat src(Size(1280,720), CV_8UC3, (void*)color_frame.get_data(), Mat::AUTO_STEP);

        Mat perspective_mat = getPerspectiveTransform(src_p, dst_p);

        // variations [W]
        //------------------------------------------------------------------------------------------
        
        // perspective matrix [W]
        warpPerspective(src, dst, perspective_mat, Size(1280,720));
        //------------------------------------------------------------------------------------------        
        // recognize image informations[W]
        width = dst.cols, height = dst.rows;
        nPoints = width * height;
        // initialization (create)[W]
        points.create(nPoints, 1, CV_32FC3);        // input data[W]
        centers.create(cluster_k, 1, points.type());        // results of k means[W]
        res.create(height, width, dst.type());      // results images[W]
        
        // data transform to fitting kmeasn algortihm[W]
        // there's no way to save computing time????[W]
        for(y = 0, n = 0; y < height; y++)
        {
            for(x = 0; x < width; x++, n++)
            {
                points.at<Vec3f>(n)[0] = dst.at<Vec3b>(y, x)[0];
                points.at<Vec3f>(n)[1] = dst.at<Vec3b>(y, x)[1];
                points.at<Vec3f>(n)[2] = dst.at<Vec3b>(y, x)[2];
            } 
        }
        
        // k-means clustering[W]
        kmeans(points, cluster_k, labels, TermCriteria(CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, 10, 1.0), 
        20, KMEANS_PP_CENTERS, centers);

        // visualization of result of kmeans algorithm[W]
        // if We don't need to visualization ...??[W]
        // test v_ 1 2022.07.12 [W]
        //---------------------------------------------------------
        for(y = 0, n = 0; y < height; y++)
        {
            for(x = 0; x < width; x++, n++)
            {
                cIndex = labels.at<int>(n);
                iTemp = cvRound(centers.at<Vec3f>(cIndex)[0]);
                iTemp = iTemp > 255 ? 255 : iTemp < 0 ? 0 : iTemp;
                res.at<Vec3b>(y, x)[0] = (uchar)iTemp;
                iTemp = cvRound(centers.at<Vec3f>(cIndex)[1]);
                iTemp = iTemp > 255 ? 255 : iTemp < 0 ? 0 : iTemp;
                res.at<Vec3b>(y, x)[1] = (uchar)iTemp;
                iTemp = cvRound(centers.at<Vec3f>(cIndex)[2]);
                iTemp = iTemp > 255 ? 255 : iTemp < 0 ? 0 : iTemp;
                res.at<Vec3b>(y, x)[2] = (uchar)iTemp;
            } 
        }
        //test
        // For remove BEV edge [HW]
        line(res, Point(0,279),Point(314,720),Scalar(0,0,0), 1); 
        line(res, Point(1280,67),Point(884,720),Scalar(0,0,0), 1);
        
        // BGR -> HSV [HW]
        cvtColor(res, hsv, COLOR_BGR2HSV);
        
        // HSV detect [W]
        // mask : color 1 is red / color 2 is blue - demo version [W]
        //========================================================================    
        if(color ==1)
        {
          inRange(hsv, lower_red, upper_red, mask);  
          inRange(hsv, lower_red_sc, upper_red_sc, mask_sc);
          bitwise_or(mask, mask_sc, mask);
          bitwise_and(res, res, image, mask);
        }
        else if (color ==2)
        {
          inRange(hsv, lower_blue, upper_blue, mask);  
          bitwise_and(res, res, image, mask);
        }
        //========================================================================
        // find contours [HW]
        findContours(mask, contours, hierachy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE); 
        drawContours(image, contours, -1, Scalar(255, 0, 0), 5);
        imwrite("contour.png", image);  
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
        int min = *min_element(y_val.begin(),y_val.end());
        int max = *max_element(y_val.begin(),y_val.end());
        int x_left = *min_element(x_val.begin(),x_val.end());
        int x_right = *max_element(x_val.begin(),x_val.end());
         
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

        // get avg of contour's bottom x value [JH]
        int sum_bottom_x = accumulate(bottom_x.begin(),bottom_x.end(),0);
        int mean_bottom_x = sum_bottom_x/bottom_x.size();
        
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
        vx = detected_line[0];
        vy = detected_line[1];
        x = int(detected_line[2]);
        y = int(detected_line[3]);
        // get (x,y) of detected line in converted coordinate [JH]
        converted_x = convert_x(x);
        converted_y = convert_y(y);
        top_y = convert_y(min);
        left_x = convert_x(x_left);
        right_x = convert_x(x_right);
        // get the source of drawing straight line [JH]
        upper_x = int(-vx/vy*(top_y-converted_y)+converted_x);
        lower_x = int(-vx/vy*(-1*converted_y) + converted_x);
        cout << "uppper x : " << upper_x << endl;
        cout << "lower x : " << lower_x << endl;
        // waypoint visualization [W]
        if(top_y > corner_threshold) // go straight
        {
            for(int i=0; i<wp_num; i++)
            {
                wp_y.push_back(top_y/wp_num*(i+1));
                cout << "wp_y _pointing wp " << i << " : " << top_y/wp_num*(i+1)<<endl;
                wp_x.push_back((-vx/vy*(wp_y[i]-converted_y)+converted_x));
                cout << "wp_x _pointing wp " << i << " : " << (vx/vy*(wp_y[i]-converted_y)+converted_x)<<endl;
            }
            // visualization representive line [W]
            line(res, Point(inv_convert_x(upper_x),inv_convert_y(top_y)), Point(inv_convert_x(lower_x),inv_convert_y(0)),Scalar(0,0,255), 3);
        }
        else if(top_y<corner_threshold) // turn right / left
        {
            for(int i=0; i<wp_num; i++) // circle waypoint y
            {
                float theta = (i+1) * wp_num * PI / 180;
                wp_y.push_back(top_y * sin(theta));
            }
            if (vy/vx > 0) // turn left - waypoint x
            {
                for(int i=0; i<wp_num; i++)
                {
                    float theta = (i+1)*wp_num * PI / 180;
                    wp_x.push_back(convert_x(mean_bottom_x)- top_y + top_y * cos(theta));
                }
            } 
            else if (vy/vx < 0) // turn right - waypoint x
            {
                for(int i=0; i<wp_num; i++)
                {
                    float theta = (i+1)*wp_num * PI / 180;
                    wp_x.push_back(convert_x(mean_bottom_x) + top_y - top_y * cos(theta));
                }
            }
        }

        for(int i=0; i<wp_num; i++)
        {
            circle(res, Point(inv_convert_x(wp_x[i]), inv_convert_y(wp_y[i])), 5, Scalar(255,255,255), 3);
        }
        
        // to pubish each waypoint [W]
        //-----------------------------------
     

        set_array(wp_set, wp_num*2);
        
        for(int i = 0; i < wp_x.size(); i++)
       
        {
            wp_set.data[2*i] = wp_x[i]*distance_of_pixel;
            wp_set.data[2*i+1] = wp_y[i]*distance_of_pixel;
        }

        waypoints_set.publish(wp_set);

        // //-----------------------------------
        
        // vector initialization [W]
        //----------------------------
        vec_delete(x_val);
        vec_delete(y_val);
        vec_delete_float(wp_x);
        vec_delete_float(wp_y);
        vec_delete(bottom_x);
        vec_delete_p(contours_sum);
        //----------------------------
        // next_time = clock();
        // how_long = (double)(next_time/CLOCKS_PER_SEC)-(double)(prev_time/CLOCKS_PER_SEC);
        
        // cout << "time : " << how_long << endl;
        // ROSINFO(ros::Time::now());

        imwrite("res.png", res);     
        imwrite("mask.png", mask);     
        // sleep(60);
        // need trouble shooting [W]
        // loop_rate.sleep();
        ros::spinOnce();
        
    }
    return 0;
}