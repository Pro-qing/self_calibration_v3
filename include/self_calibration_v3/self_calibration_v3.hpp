#ifndef SELF_CALIBRATION_V3_HPP
#define SELF_CALIBRATION_V3_HPP

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h> // 必须包含：用于点云变换
#include <Eigen/Dense>
#include <nav_msgs/Odometry.h>
#include <vector>
#include <string>
#include <deque>
#include <fstream> 

struct LaserLine {
    float avg_distance; 
    Eigen::Vector3f normal; 
    std::string label;
    float confidence;
    int point_count;
    std::vector<pcl::PointXYZ> raw_points;
    LaserLine() : avg_distance(0), confidence(0), point_count(0) {}
};

class LidarCalibration
{
public:
    LidarCalibration(ros::NodeHandle& nh);
    void run();
    
private:
    ros::NodeHandle nh_;
    ros::Subscriber cloud_sub_, mid_cloud_sub_;
    // 新增 transformed_mid_pub_
    ros::Publisher filtered_pub_, mid_filtered_pub_, wall_points_pub_, mid_wall_pub_, transformed_mid_pub_;

    std::string lidar_frame_, base_frame_, mid_frame_;
    float actual_left_dist_, actual_right_dist_, actual_front_dist_;
    float manual_lidar_height_;

    Eigen::Matrix4f T_base_to_velo_;
    Eigen::Matrix4f T_base_to_mid_;
    bool velo_ready_;
    
    // --- 平滑处理相关变量 ---
    std::deque<Eigen::Matrix4f> calibration_queue_;
    const size_t SMOOTH_WINDOW_SIZE = 50;
    
    LaserLine velo_left_, velo_right_, velo_front_;
    LaserLine mid_ground_, mid_left_, mid_right_, mid_front_;

    void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);
    void midPointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);
    
    void calculateVeloToBase();
    void calculateMidToBase();
    
    Eigen::Matrix4f getSmoothedResult();
    
    bool detectThreePlanes(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, 
                          LaserLine& left, LaserLine& right, LaserLine& front, bool is_mid);
    bool detectGround(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, LaserLine& ground);

    void printFinalResult(const Eigen::Matrix4f& T);
    
    // 新增：发布转换后的点云
    void publishTransformedMidCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, 
                                   const Eigen::Matrix4f& T_mid_to_velo);

    // [新增] 用于保存 YAML 文件的路径和函数
    std::string save_path_;
    void saveToYAML(const Eigen::Matrix4f& T, const std::string& lidar_calibration);
    
    float toRadians(float d);
    float toDegrees(float r);
};

#endif