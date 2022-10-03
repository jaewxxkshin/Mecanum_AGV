#include <ros/ros.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <cstdio>
#include <tf2/LinearMath/Quaternion.h>
#include <cmath>
#define PI 3.141592

int main(int argc, char **argv){
    ros::init(argc,argv,"setting_d435_world");

    static tf2_ros::StaticTransformBroadcaster static_broadcaster;
    geometry_msgs::TransformStamped static_transformStamped;

    static_transformStamped.header.stamp = ros::Time::now();
    static_transformStamped.header.frame_id="camera_odom_frame";
    static_transformStamped.child_frame_id="d435_world";
    static_transformStamped.transform.translation.x=-0.3;
    static_transformStamped.transform.translation.y=0.025;
    static_transformStamped.transform.translation.z=0;
    tf2::Quaternion quat;
    // rearrange origin of coordimate
    quat.setRPY(0.0, 0.0, 270*PI/180);
    static_transformStamped.transform.rotation.x=quat.x();
    static_transformStamped.transform.rotation.y=quat.y();
    static_transformStamped.transform.rotation.z=quat.z();
    static_transformStamped.transform.rotation.w=quat.w();

    static_broadcaster.sendTransform(static_transformStamped);
    ros::spin();
    return 0;
}