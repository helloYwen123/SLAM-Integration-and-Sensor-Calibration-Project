#include "ros/ros.h"
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

int main(int argc, char** argv){
    ros::init(argc, argv, "tf_listener_node");
    ros::NodeHandle nh;

    
    ros::Publisher pose_pub = nh.advertise<geometry_msgs::PointStamped>("pose", 10);

   
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);

    ros::Rate rate(10.0);
    while (nh.ok()){
        geometry_msgs::TransformStamped transformStamped;
        try{
            
            transformStamped = tfBuffer.lookupTransform("map", "robot_base_link", ros::Time(0));

            
            geometry_msgs::PointStamped pose;
            pose.header.stamp = ros::Time::now();
            pose.header.frame_id = "map";
            pose.point.x = transformStamped.transform.translation.x;
            pose.point.y = transformStamped.transform.translation.y;

            
            tf2::Quaternion q(
                transformStamped.transform.rotation.x,
                transformStamped.transform.rotation.y,
                transformStamped.transform.rotation.z,
                transformStamped.transform.rotation.w);
            tf2::Matrix3x3 m(q);
            double roll, pitch, yaw;
            m.getRPY(roll, pitch, yaw);
            
            pose_pub.publish(pose);

        }
        catch (tf2::TransformException &ex) {
            ROS_WARN("%s",ex.what());
            ros::Duration(1.0).sleep();
            continue;
        }
        rate.sleep();
    }
    return 0;
}
