
#include "multi_lidar_calibration.h"
#include <chrono>



MultiLidarCalibration::MultiLidarCalibration(ros::NodeHandle &n) : nh_(n)
{
    ROS_INFO_STREAM("\033[1;32m----> Multi Lidar Calibration Use NDT...\033[0m");

    nh_.param<std::string>("/multi_lidar_calibration_node/source_lidar_topic", source_lidar_topic_str_, "/sick_back/scan");
    nh_.param<std::string>("/multi_lidar_calibration_node/target_lidar_topic", target_lidar_topic_str_, "/sick_front/scan");
    nh_.param<std::string>("/multi_lidar_calibration_node/source_lidar_frame", source_lidar_frame_str_, "sub_laser_link");
    nh_.param<std::string>("/multi_lidar_calibration_node/target_lidar_frame", target_lidar_frame_str_, "main_laser_link");
    nh_.param<float>("/multi_lidar_calibration_node/ndt_score", ndt_score_, 5.5487);
    

    // 发布转换后的激光点云
    final_point_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/final_point_cloud", 10);
    sub_point_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/sub_point_cloud", 10);
    main_point_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/main_point_cloud", 10);
    sub_transformed_point_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/sub_transformed_point_cloud", 10);

    // 订阅多个激光话题
    scan_front_subscriber_ = new message_filters::Subscriber<sensor_msgs::LaserScan>(nh_, target_lidar_topic_str_, 1);
    scan_back_subscriber_ = new message_filters::Subscriber<sensor_msgs::LaserScan>(nh_, source_lidar_topic_str_, 1);
    scan_synchronizer_ = new message_filters::Synchronizer<SyncPolicyT>(SyncPolicyT(10), *scan_front_subscriber_, *scan_back_subscriber_);
    scan_synchronizer_->registerCallback(boost::bind(&MultiLidarCalibration::ScanCallBack, this, _1, _2));
    
    


    // 参数赋值
    is_first_run_ = true;

    // 在main_laser_link下sub_laser_link的坐标
    transform_martix_ = Eigen::Matrix4f::Identity(); //4 * 4 齐次坐标
    // 在base_link坐标系下main_laser_link的坐标
    front_to_base_link_ = Eigen::Matrix4f::Identity();
    
    main_scan_pointcloud_ = boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>(new pcl::PointCloud<pcl::PointXYZ>());
    sub_scan_pointcloud_ = boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>(new pcl::PointCloud<pcl::PointXYZ>());
    final_registration_scan_ = boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>(new pcl::PointCloud<pcl::PointXYZ>());
    
    sub_scan_pointcloud_init_transformed_ = boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>(new pcl::PointCloud<pcl::PointXYZ>());
    if (!main_scan_pointcloud_ || !sub_scan_pointcloud_ || !final_registration_scan_ || !sub_scan_pointcloud_init_transformed_) {
            ROS_ERROR("One of the point cloud pointers is null!");
    }

}

MultiLidarCalibration::~MultiLidarCalibration() {//qRegisterMetaType<QVector<int>>("QVector<int>");
}

/**
 * @brief 获取激光雷达间的坐标变换
 * 
 * @param transform_martix_ 激光雷达间的转换矩阵
 * @param front_to_base_link_ 在main_laser_link下sub_laser_link的坐标
 */
void MultiLidarCalibration::GetBackLasertoFrontLaserTf()
{
    tf2_ros::Buffer buffer;
    tf2_ros::TransformListener tfl(buffer);
    

    ros::Time time = ros::Time::now();
    ros::Duration timeout(0.5);
    timeout.sleep();
    geometry_msgs::TransformStamped tfGeom;
    
    try
    {
        tfGeom = buffer.lookupTransform(source_lidar_frame_str_,target_lidar_frame_str_, ros::Time::now(), ros::Duration(3.0));
    }
    catch (tf2::TransformException &e)
    {
        ROS_ERROR_STREAM("Lidar Transform Error ... ");
    }

    

    // tf2矩阵转换成Eigen::Matrix4f
    Eigen::Quaternionf qw(tfGeom.transform.rotation.w, tfGeom.transform.rotation.x, tfGeom.transform.rotation.y, tfGeom.transform.rotation.z); //tf 获得的四元数
    Eigen::Vector3f qt(tfGeom.transform.translation.x, tfGeom.transform.translation.y, tfGeom.transform.translation.z);                        //tf获得的平移向量
    transform_martix_.block<3, 3>(0, 0) = qw.toRotationMatrix();
    transform_martix_.block<3, 1>(0, 3) = qt;

    
        ROS_INFO_STREAM("from rear to front link=\n"
                    << transform_martix_);

}

/**
  * @brief 激光雷达发布点云
  * @param in_cloud_to_publish_ptr 输入icp转换后的激光点云数据
  */
void MultiLidarCalibration::PublishCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr &in_cloud_to_publish_ptr)
{
    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(*in_cloud_to_publish_ptr, cloud_msg);
    
    cloud_msg.header.frame_id = target_lidar_frame_str_; //source_lidar_frame_str_;//target_lidar_frame_str_;
    final_point_cloud_pub_.publish(cloud_msg);
}

void MultiLidarCalibration::PublishCloud_sub(pcl::PointCloud<pcl::PointXYZ>::Ptr &in_cloud_to_publish_ptr)
{
    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(*in_cloud_to_publish_ptr, cloud_msg);
    cloud_msg.header.frame_id = source_lidar_frame_str_;
    sub_point_cloud_pub_.publish(cloud_msg);
}

void MultiLidarCalibration::PublishCloud_main(pcl::PointCloud<pcl::PointXYZ>::Ptr &in_cloud_to_publish_ptr)
{
    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(*in_cloud_to_publish_ptr, cloud_msg);
    cloud_msg.header.frame_id = target_lidar_frame_str_;
    main_point_cloud_pub_.publish(cloud_msg);
}

void MultiLidarCalibration::PublishCloud_sub_transformed(pcl::PointCloud<pcl::PointXYZ>::Ptr &in_cloud_to_publish_ptr)
{
    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(*in_cloud_to_publish_ptr, cloud_msg);
    cloud_msg.header.frame_id = target_lidar_frame_str_;
    sub_transformed_point_cloud_pub_.publish(cloud_msg);
}

void MultiLidarCalibration::filterByDistance(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered, double max_distance)
{
    
    pcl::ConditionAnd<pcl::PointXYZ>::Ptr range_cond (new pcl::ConditionAnd<pcl::PointXYZ>());
    range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new
        pcl::FieldComparison<pcl::PointXYZ> ("x", pcl::ComparisonOps::LT, max_distance)));
    range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new
        pcl::FieldComparison<pcl::PointXYZ> ("x", pcl::ComparisonOps::GT, -max_distance)));
    range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new
        pcl::FieldComparison<pcl::PointXYZ> ("y", pcl::ComparisonOps::LT, max_distance)));
    range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new
        pcl::FieldComparison<pcl::PointXYZ> ("y", pcl::ComparisonOps::GT, -max_distance)));

    
    pcl::ConditionalRemoval<pcl::PointXYZ> condrem;
    condrem.setCondition(range_cond);
    condrem.setInputCloud(cloud);
    condrem.setKeepOrganized(true);
    condrem.filter(*cloud_filtered);
}




/**
 * @brief 激光雷达消息类型转换 sensor_msg::Laser to pcl::PointCloud<pcl::PointXYZ>
 * 
 * @param scan_msg 输入sensor_msgs
 * @return pcl::PointCloud<pcl::PointXYZ> 输出pcl格式点云
 */
pcl::PointCloud<pcl::PointXYZ> MultiLidarCalibration::ConvertScantoPointCloud(const sensor_msgs::LaserScan::ConstPtr &scan_msg)
{
    pcl::PointCloud<pcl::PointXYZ> cloud_points;
    pcl::PointXYZ points;

    for (int i = 0; i < scan_msg->ranges.size(); ++i)
    {
        float range = scan_msg->ranges[i];
        if (!std::isfinite(range))
        {
            continue;
        }

        if (range > scan_msg->range_min && range < scan_msg->range_max)
        //if (range > scan_msg->range_min && range < 3)
        {
            float angle = scan_msg->angle_min + i * scan_msg->angle_increment;
            angle = angle;
            points.x = range * cos(angle);
            points.y = range * sin(angle);
            points.z = 0.0;
            cloud_points.push_back(points);
        }
    }
    return cloud_points;
}

/**
 * @brief 多个激光雷达数据同步
 * 
 * @param in_main_scan_msg 激光雷达topic 1
 * @param in_sub_scan_msg 激光雷达topic 2
 */ 
// main : front ; sub: rear
void MultiLidarCalibration::ScanCallBack(const sensor_msgs::LaserScan::ConstPtr &in_main_scan_msg, const sensor_msgs::LaserScan::ConstPtr &in_sub_scan_msg)
{

    main_scan_pointcloud_ = ConvertScantoPointCloud(in_main_scan_msg).makeShared();
    sub_scan_pointcloud_ = ConvertScantoPointCloud(in_sub_scan_msg).makeShared();

    if (!main_scan_pointcloud_ || !sub_scan_pointcloud_) {
        ROS_ERROR("Failed to convert scan to point cloud!");
        return;
    }
}

/**
 * @brief 两个激光雷达数据进行icp匹配
 * 
 */
bool MultiLidarCalibration::ScanRegistration()
{
    if (0 == main_scan_pointcloud_->points.size() || 0 == sub_scan_pointcloud_->points.size())
    {
        return false;
    }

    // transformed point means the filtered points from sub_scan_pointcloud
    approximate_voxel_filter.setLeafSize(0.1, 0.1, 0.1); 
    approximate_voxel_filter.setInputCloud(sub_scan_pointcloud_);
    approximate_voxel_filter.filter(*sub_scan_pointcloud_init_transformed_);
    std::cout << "Filtered cloud contains " << sub_scan_pointcloud_init_transformed_ -> size() << " data points from scan.pcd" << std::endl;

    ndt.setTransformationEpsilon(0.001);
    ndt.setStepSize(0.02);
    ndt.setResolution(1.0);
    ndt.setMaximumIterations(100);
    ndt.setInputSource(sub_scan_pointcloud_init_transformed_);
    ndt.setInputTarget(main_scan_pointcloud_);
    ndt.align(*final_registration_scan_, transform_martix_);
    
    if (ndt.hasConverged() == false && ndt.getFitnessScore() > 1.0)
    {
        ROS_WARN_STREAM("Not Converged ... ");
        return false;
    }


    return true;
}

/**
 * @brief 打印结果 
 * 
 */
void MultiLidarCalibration::PrintResult()
{
    ROS_INFO_STREAM("fitness_score: \n" << ndt.getFitnessScore());
    if (ndt.getFitnessScore() > ndt_score_)
    {
        ROS_INFO_STREAM("fitnessScore is too big! the registrated results are too bad!!\n");
        return;
    }
    
    // sub激光雷达到main雷达的icp的计算结果 source to target rear -> front
    Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
    T = ndt.getFinalTransformation();

    pcl::transformPointCloud(*sub_scan_pointcloud_, *final_registration_scan_, T);

    ROS_INFO_STREAM("T=\n"
                    << T);
    Eigen::Matrix3f R3 = T.block<3, 3>(0, 0);
    Eigen::Vector3f t3 = T.block<3, 1>(0, 3);

    Eigen::Matrix3f R4 = transform_martix_.block<3, 3>(0, 0);
    Eigen::Vector3f t4 = transform_martix_.block<3, 1>(0, 3);
    ROS_INFO_STREAM("initial Transform matrix=\n"
                    << transform_martix_);

    // 输出转换关系
    Eigen::Vector3f eulerAngle = R3.eulerAngles(0,1,2);
    ROS_INFO_STREAM("compute the angle and vector: \n");
    ROS_INFO_STREAM("eulerAngle=\n"
                    << eulerAngle);
    ROS_INFO_STREAM("transform vector=\n"
                  << t3);
}



/**
 * @brief 运行主函数
 * 
 */
void MultiLidarCalibration::Run()
{
    if (is_first_run_)
    {
        GetBackLasertoFrontLaserTf();
        is_first_run_ = false;
        return;
    }
    
    PublishCloud_sub(sub_scan_pointcloud_);
    PublishCloud_main(main_scan_pointcloud_);
   

    // 进行ndt匹配，匹配失败返回
    if (!ScanRegistration())
    {
        return;
    }
    
    //PublishCloud_sub_transformed(sub_scan_pointcloud_init_transformed_);
    PrintResult();
    PublishCloud(final_registration_scan_);
    
    

    

    //View();
}
