
#ifndef MULTI_LIDAR_CALIBRATION_H_
#define MULTI_LIDAR_CALIBRATION_H_

#include <iostream>
#include <vector>

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/TransformStamped.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <tf2/utils.h>
#include <tf2_ros/transform_listener.h>

#include <pcl_ros/point_cloud.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>


#include <pcl/registration/ndt.h>  
#include <pcl/filters/approximate_voxel_grid.h> 

#include <pcl/io/ply_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/conditional_removal.h>

class MultiLidarCalibration
{
public:
    MultiLidarCalibration(ros::NodeHandle &n);
    ~MultiLidarCalibration();

    
    void Run();

    void customTransformPointCloud(
    const pcl::PointCloud<pcl::PointXYZ>& input,
    pcl::PointCloud<pcl::PointXYZ>& output,
    const Eigen::Matrix4f& transform_matrix);

private:

    std::string source_lidar_topic_str_;
    std::string target_lidar_topic_str_;

    
    std::string source_lidar_frame_str_;
    std::string target_lidar_frame_str_;

    
    float ndt_score_;

    
    float main_to_base_transform_x_;
    float main_to_base_transform_y_;
    float main_to_base_transform_row_;
    float main_to_base_transform_yaw_;

    
    bool is_first_run_;

   
    Eigen::Matrix4f transform_martix_;
    Eigen::Matrix4f transform_martix2_;
   
    Eigen::Matrix4f front_to_base_link_;

    ros::NodeHandle nh_;

 
    ros::Publisher final_point_cloud_pub_;
    ros::Publisher sub_point_cloud_pub_;
    ros::Publisher main_point_cloud_pub_;
    ros::Publisher sub_transformed_point_cloud_pub_;

  
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::LaserScan, sensor_msgs::LaserScan> SyncPolicyT;
    message_filters::Subscriber<sensor_msgs::LaserScan> *scan_front_subscriber_, *scan_back_subscriber_;
    message_filters::Synchronizer<SyncPolicyT> *scan_synchronizer_;


    pcl::PointCloud<pcl::PointXYZ>::Ptr main_scan_pointcloud_;

    pcl::PointCloud<pcl::PointXYZ>::Ptr sub_scan_pointcloud_;

    pcl::PointCloud<pcl::PointXYZ>::Ptr sub_scan_pointcloud_init_transformed_;

    /**
     * @brief icp 
     * @param final_registration_scan_ 
     */

    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp_;
    
    pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt;

    pcl::ApproximateVoxelGrid<pcl::PointXYZ> approximate_voxel_filter;



    pcl::PointCloud<pcl::PointXYZ>::Ptr final_registration_scan_;


    void GetBackLasertoFrontLaserTf();


    void ScanCallBack(const sensor_msgs::LaserScan::ConstPtr &in_main_scan_msg, const sensor_msgs::LaserScan::ConstPtr &in_sub_scan_msg);

    pcl::PointCloud<pcl::PointXYZ> ConvertScantoPointCloud(const sensor_msgs::LaserScan::ConstPtr &scan_msg);

    void PublishCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr &in_cloud_to_publish_ptr);
    void PublishCloud_sub(pcl::PointCloud<pcl::PointXYZ>::Ptr &in_cloud_to_publish_ptr);
    void PublishCloud_main(pcl::PointCloud<pcl::PointXYZ>::Ptr &in_cloud_to_publish_ptr);
    void PublishCloud_sub_transformed(pcl::PointCloud<pcl::PointXYZ>::Ptr &in_cloud_to_publish_ptr);

    
    bool ScanRegistration();


    void PrintResult();

    void filterByDistance(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered, double max_distance);
    void View();
};

#endif //MULTI_LIDAR_CALIBRATION_H_
