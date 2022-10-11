#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>

int main(int argc, char** argv){
    ros::init(argc,argv,"tf_listener");

    ros::NodeHandle nh;

    ros::Publisher pos=nh.advertise<geometry_msgs::Vector3>("/d435_pos",100);
    ros::Publisher rot=nh.advertise<geometry_msgs::Quaternion>("/d435_rot",100);

    ros::Publisher platform_pos=nh.advertise<geometry_msgs::Vector3>("/platform_pos",100);
    ros::Publisher platform_rot=nh.advertise<geometry_msgs::Quaternion>("/platform_rot",100);

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);

    tf2_ros::Buffer tfBuffer2;
    tf2_ros::TransformListener tfListener2(tfBuffer2);
 

    ros::Rate rate(100);
    while(nh.ok()){
        geometry_msgs::TransformStamped transformStamped;
        geometry_msgs::TransformStamped transformStamped2;

        try{
            // argv[0] & argv[1]'s order trouble [W] 
            transformStamped = tfBuffer.lookupTransform("d435_world","d435_frame",ros::Time(0));
            transformStamped2 = tfBuffer2.lookupTransform("camera_odom_frame","platform_frame",ros::Time(0));
        }
        catch(tf2::TransformException &ex){
            ROS_WARN("%s",ex.what());
            ros::Duration(1.0).sleep();
            continue;
        }

        geometry_msgs::Vector3 trans;
        geometry_msgs::Quaternion quat;

        geometry_msgs::Vector3 trans2;
        geometry_msgs::Quaternion quat2;

        trans.x=transformStamped.transform.translation.x;
        trans.y=transformStamped.transform.translation.y;
        trans.z=transformStamped.transform.translation.z;

        quat.x=transformStamped.transform.rotation.x;
        quat.y=transformStamped.transform.rotation.y;
        quat.z=transformStamped.transform.rotation.z;
        quat.w=transformStamped.transform.rotation.w;

        trans2.x=transformStamped2.transform.translation.x;
        trans2.y=transformStamped2.transform.translation.y;
        trans2.z=transformStamped2.transform.translation.z;

        quat2.x=transformStamped2.transform.rotation.x;
        quat2.y=transformStamped2.transform.rotation.y;
        quat2.z=transformStamped2.transform.rotation.z;
        quat2.w=transformStamped2.transform.rotation.w;

        pos.publish(trans);
        rot.publish(quat);
        platform_pos.publish(trans2);
        platform_rot.publish(quat2);


        rate.sleep();
    }
    return 0;
}