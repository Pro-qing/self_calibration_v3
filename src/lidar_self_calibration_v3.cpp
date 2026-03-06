#include "self_calibration_v3/self_calibration_v3.hpp"
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/common/common.h>
#include <pcl/common/centroid.h>
#include <pcl/common/transforms.h> // 确保包含变换库
#include <clocale>
#include <cmath>
#include <iomanip>

// 构造函数
LidarCalibration::LidarCalibration(ros::NodeHandle& nh) : nh_(nh)
{
    std::setlocale(LC_ALL, ""); 
    
    // 参数读取
    nh_.param<std::string>("lidar_frame", lidar_frame_, "velodyne");
    nh_.param<std::string>("base_frame", base_frame_, "base_link");
    nh_.param<std::string>("mid_frame", mid_frame_, "mid_lidar");
    
    // [新增] 读取 YAML 文件保存路径
    nh_.param<std::string>("save_path", save_path_, "~/laser_self_calibration_workspace/src/self_calibration_v3/param/lidar_calibration.yaml");

    nh_.param<float>("actual_left_distance", actual_left_dist_, 0.0f);   
    nh_.param<float>("actual_right_distance", actual_right_dist_, 0.0f);
    nh_.param<float>("actual_front_distance", actual_front_dist_, 0.0f); 
    nh_.param<float>("manual_lidar_height", manual_lidar_height_, 1.0f);

    cloud_sub_ = nh_.subscribe<sensor_msgs::PointCloud2>("/points_16", 1, &LidarCalibration::pointCloudCallback, this);
    mid_cloud_sub_ = nh_.subscribe<sensor_msgs::PointCloud2>("/points_mid", 1, &LidarCalibration::midPointCloudCallback, this);
    // mid_cloud_sub_ = nh_.subscribe<sensor_msgs::PointCloud2>("/points_mid", 1, &LidarCalibration::midPointCloudCallback, this);

    filtered_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("filtered_points", 1);
    wall_points_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("wall_points", 1);
    
    mid_filtered_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("mid_filtered_points", 1);
    mid_wall_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("mid_wall_points", 1);
    
    // [新增] 用于RViz验证的发布器
    transformed_mid_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/velodyne_points_mid", 1);
    
    T_base_to_velo_.setIdentity(); 
    T_base_to_mid_.setIdentity();  
    velo_ready_ = false;
    
    calibration_queue_.clear();
    
    ROS_INFO("================================================");
    ROS_INFO("   双雷达标定 (可视化验证版)");
    ROS_INFO("   RViz话题: /velodyne_points_mid (Frame: velodyne)");
    ROS_INFO("================================================");
}

// --------------------------------------------------------------------------------
// 1. Velodyne -> Base_Link 计算
// --------------------------------------------------------------------------------
void LidarCalibration::pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg, *cloud);
    if (cloud->empty()) return;

    pcl::PointCloud<pcl::PointXYZ>::Ptr z_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(cloud);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(-1.0, 1.0); 
    pass.filter(*z_filtered);

    filtered_pub_.publish(msg); 

    if (detectThreePlanes(z_filtered, velo_left_, velo_right_, velo_front_, false)) {
        calculateVeloToBase();
        velo_ready_ = true;
        
        pcl::PointCloud<pcl::PointXYZRGB> colored;
        auto add = [&](const LaserLine& l, int r, int g, int b) {
            for(auto p : l.raw_points) {
                pcl::PointXYZRGB cp; cp.x=p.x; cp.y=p.y; cp.z=p.z; cp.r=r; cp.g=g; cp.b=b; colored.push_back(cp);
            }
        };
        add(velo_left_, 255,0,0); add(velo_right_, 0,255,0); add(velo_front_, 0,0,255);
        sensor_msgs::PointCloud2 out; pcl::toROSMsg(colored, out);
        out.header = msg->header; wall_points_pub_.publish(out);
    }
}

void LidarCalibration::calculateVeloToBase()
{
    Eigen::Vector3f x_axis_base_in_velo = -velo_front_.normal;
    Eigen::Vector3f y_axis_base_in_velo;
    
    float conf_sum = velo_left_.confidence + velo_right_.confidence;
    if (conf_sum > 0) {
        // Y轴反转修复逻辑保持
        y_axis_base_in_velo = (-velo_left_.normal * velo_left_.confidence + velo_right_.normal * velo_right_.confidence) / conf_sum;
    } else {
        y_axis_base_in_velo = -velo_left_.normal;
    }

    y_axis_base_in_velo = (y_axis_base_in_velo - (y_axis_base_in_velo.dot(x_axis_base_in_velo)) * x_axis_base_in_velo).normalized();
    Eigen::Vector3f z_axis_base_in_velo = x_axis_base_in_velo.cross(y_axis_base_in_velo).normalized();

    Eigen::Matrix3f R_base_in_velo;
    R_base_in_velo.col(0) = x_axis_base_in_velo;
    R_base_in_velo.col(1) = y_axis_base_in_velo;
    R_base_in_velo.col(2) = z_axis_base_in_velo;
    
    Eigen::Matrix3f R_velo_in_base = R_base_in_velo.transpose();

    float tx = actual_front_dist_ - velo_front_.avg_distance;
    
    float ty_from_left = actual_left_dist_ - velo_left_.avg_distance;
    float ty_from_right = -actual_right_dist_ + velo_right_.avg_distance; 
    
    float ty = 0.0f;
    if (conf_sum > 0) {
        ty = (ty_from_left * velo_left_.confidence + ty_from_right * velo_right_.confidence) / conf_sum;
    } else {
        ty = ty_from_left;
    }
    
    float tz = manual_lidar_height_;

    T_base_to_velo_.setIdentity();
    T_base_to_velo_.block<3,3>(0,0) = R_velo_in_base;
    T_base_to_velo_(0,3) = tx;
    T_base_to_velo_(1,3) = ty;
    T_base_to_velo_(2,3) = tz;
}

// --------------------------------------------------------------------------------
// 2. Mid -> Base_Link 计算
// --------------------------------------------------------------------------------
void LidarCalibration::midPointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
    if (!velo_ready_) return;

    // [关键] 保留一份原始点云用于变换发布
    pcl::PointCloud<pcl::PointXYZ>::Ptr raw_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg, *raw_cloud);
    
    if (raw_cloud->empty()) return;

    // 处理用的副本
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>(*raw_cloud));

    // 1. 距离滤波
    pcl::PointCloud<pcl::PointXYZ>::Ptr dist_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    dist_filtered->header = cloud->header;
    const float MIN_RANGE = 1.0f; 

    for (const auto& p : cloud->points) {
        float range = std::sqrt(p.x*p.x + p.y*p.y + p.z*p.z);
        if (range > MIN_RANGE) {
            dist_filtered->push_back(p);
        }
    }
    if (dist_filtered->empty()) return;
    cloud = dist_filtered; 

    // 2. 地面检测
    if (!detectGround(cloud, mid_ground_)) return;

    // 3. 墙面滤波
    pcl::PointCloud<pcl::PointXYZ>::Ptr wall_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(cloud);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(-10.0, mid_ground_.avg_distance - 0.2); 
    pass.filter(*wall_cloud);

    sensor_msgs::PointCloud2 debug_msg;
    pcl::toROSMsg(*wall_cloud, debug_msg);
    debug_msg.header = msg->header;
    mid_filtered_pub_.publish(debug_msg);

    // 4. 三面墙检测 & 标定
    if (detectThreePlanes(wall_cloud, mid_left_, mid_right_, mid_front_, true)) {
        
        calculateMidToBase();
        
        // 计算瞬时外参 T_mid -> velo
        Eigen::Matrix4f T_instant = T_base_to_velo_.inverse() * T_base_to_mid_;
        
        // 加入队列
        calibration_queue_.push_back(T_instant);
        if (calibration_queue_.size() > SMOOTH_WINDOW_SIZE) {
            calibration_queue_.pop_front();
        }

        // 获取平滑结果
        Eigen::Matrix4f T_final = getSmoothedResult();
        
        printFinalResult(T_final);

        // ========================================================
        // [新增] 当平滑窗口满了之后，将结果实时覆盖写入 YAML 文件
        // ========================================================
        if (calibration_queue_.size() >= SMOOTH_WINDOW_SIZE) {
            saveToYAML(T_final, save_path_);
        }

        
        // ========================================================
        // [新增] 发布转换后的点云到 /velodyne_points_mid
        // ========================================================
        // 使用 raw_cloud (包含地面) 进行变换，方便在 RViz 中观察地面是否对齐
        publishTransformedMidCloud(raw_cloud, T_final);

        // 可视化提取的平面
        pcl::PointCloud<pcl::PointXYZRGB> colored;
        auto add = [&](const LaserLine& l, int r, int g, int b) {
            for(auto p : l.raw_points) {
                pcl::PointXYZRGB cp; cp.x=p.x; cp.y=p.y; cp.z=p.z; 
                uint32_t rgb = (static_cast<uint32_t>(r) << 16 | static_cast<uint32_t>(g) << 8 | static_cast<uint32_t>(b));
                cp.rgb = *reinterpret_cast<float*>(&rgb);
                colored.push_back(cp);
            }
        };
        add(mid_ground_, 250,250,0);
        add(mid_left_, 255,0,0);
        add(mid_right_, 0,255,0);
        add(mid_front_, 0,0,255);
        sensor_msgs::PointCloud2 out; pcl::toROSMsg(colored, out);
        out.header = msg->header; mid_wall_pub_.publish(out);
    }
}

// 点云变换发布函数
void LidarCalibration::publishTransformedMidCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, 
                                                 const Eigen::Matrix4f& T_mid_to_velo)
{
    if (cloud->empty()) return;

    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    
    // 执行刚体变换: P_velo = T * P_mid
    pcl::transformPointCloud(*cloud, *transformed_cloud, T_mid_to_velo);
    
    sensor_msgs::PointCloud2 output_msg;
    pcl::toROSMsg(*transformed_cloud, output_msg);
    
    // 将 Frame ID 修改为目标坐标系 (velodyne)
    // 这样在 RViz 中 Fixed Frame 选 velodyne 时，两份点云才能正确叠加
    output_msg.header.frame_id = lidar_frame_; 
    output_msg.header.stamp = ros::Time::now();
    
    transformed_mid_pub_.publish(output_msg);
}

Eigen::Matrix4f LidarCalibration::getSmoothedResult()
{
    if (calibration_queue_.empty()) return Eigen::Matrix4f::Identity();

    Eigen::Vector3f sum_t(0, 0, 0);
    Eigen::Vector4f sum_q(0, 0, 0, 0);
    Eigen::Quaternionf q_ref(calibration_queue_.front().block<3,3>(0,0));

    for (const auto& T : calibration_queue_) {
        sum_t += T.block<3,1>(0,3);
        Eigen::Quaternionf q_curr(T.block<3,3>(0,0));
        if (q_curr.dot(q_ref) < 0) {
            q_curr.coeffs() = -q_curr.coeffs();
        }
        sum_q += q_curr.coeffs();
    }

    float n = static_cast<float>(calibration_queue_.size());
    sum_t /= n;
    sum_q.normalize();

    Eigen::Matrix4f T_smooth = Eigen::Matrix4f::Identity();
    T_smooth.block<3,3>(0,0) = Eigen::Quaternionf(sum_q).toRotationMatrix();
    T_smooth.block<3,1>(0,3) = sum_t;

    return T_smooth;
}

void LidarCalibration::calculateMidToBase()
{
    Eigen::Vector3f z_axis_base_in_mid = mid_ground_.normal;
    Eigen::Vector3f x_axis_base_in_mid = -mid_front_.normal;
    
    x_axis_base_in_mid = (x_axis_base_in_mid - (x_axis_base_in_mid.dot(z_axis_base_in_mid)) * z_axis_base_in_mid).normalized();
    Eigen::Vector3f y_axis_base_in_mid = z_axis_base_in_mid.cross(x_axis_base_in_mid).normalized();
    
    Eigen::Matrix3f R_base_in_mid;
    R_base_in_mid.col(0) = x_axis_base_in_mid;
    R_base_in_mid.col(1) = y_axis_base_in_mid;
    R_base_in_mid.col(2) = z_axis_base_in_mid;
    
    Eigen::Matrix3f R_mid_in_base = R_base_in_mid.transpose();
    
    float tx = actual_front_dist_ - mid_front_.avg_distance;
    
    float ty_from_left = actual_left_dist_ - mid_left_.avg_distance;
    float ty_from_right = mid_right_.avg_distance - actual_right_dist_;
    
    float total_conf = mid_left_.confidence + mid_right_.confidence;
    float ty = 0.0f;
    if (total_conf > 0) {
        ty = (ty_from_left * mid_left_.confidence + ty_from_right * mid_right_.confidence) / total_conf;
    } else {
        ty = ty_from_left; 
    }
    
    float tz = mid_ground_.avg_distance;

    T_base_to_mid_.setIdentity();
    T_base_to_mid_.block<3,3>(0,0) = R_mid_in_base;
    T_base_to_mid_(0,3) = tx;
    T_base_to_mid_(1,3) = ty;
    T_base_to_mid_(2,3) = tz;
}

bool LidarCalibration::detectGround(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, LaserLine& ground)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cand(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(cloud);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(1.0, 3.0); 
    pass.filter(*cand);
    
    if (cand->size() < 100) return false;

    pcl::ModelCoefficients::Ptr coeff(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(0.05);
    seg.setInputCloud(cand);
    seg.segment(*inliers, *coeff);

    if (inliers->indices.size() < 100) return false;

    Eigen::Vector3f n(coeff->values[0], coeff->values[1], coeff->values[2]);
    if (n.z() > 0) n = -n; 

    ground.avg_distance = std::abs(coeff->values[3]);
    ground.normal = n; 
    ground.point_count = inliers->indices.size();
    
    ground.raw_points.clear();
    for(int idx : inliers->indices) ground.raw_points.push_back(cand->points[idx]);
    
    return true;
}

bool LidarCalibration::detectThreePlanes(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                                        LaserLine& left, LaserLine& right, LaserLine& front,
                                        bool is_mid_lidar)
{
    if (cloud->empty()) return false;
    
    struct Region { std::string label; float min_a; float max_a; };
    std::vector<Region> regions;
    
    if (!is_mid_lidar) {
        regions = { {"front", -45, 45}, {"left", 45, 135}, {"right", -135, -45} };
    } else {
        regions = { {"front", -45, 45}, {"right", 45, 135}, {"left", -135, -45} };
    }

    bool has_l=false, has_r=false, has_f=false;

    for (const auto& reg : regions) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr roi(new pcl::PointCloud<pcl::PointXYZ>);
        float min_rad = toRadians(reg.min_a);
        float max_rad = toRadians(reg.max_a);

        for (const auto& p : cloud->points) {
            float r = std::sqrt(p.x*p.x + p.y*p.y);
            if(r < 0.5 || r > 10.0) continue;
            float ang = std::atan2(p.y, p.x);
            if (ang >= min_rad && ang <= max_rad) roi->push_back(p);
        }

        if (roi->size() < 30) continue;

        pcl::ModelCoefficients::Ptr coeff(new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
        pcl::SACSegmentation<pcl::PointXYZ> seg;
        seg.setModelType(pcl::SACMODEL_PLANE);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setDistanceThreshold(0.05);
        seg.setInputCloud(roi);
        seg.segment(*inliers, *coeff);

        if (inliers->indices.size() < 30) continue;

        Eigen::Vector3f n(coeff->values[0], coeff->values[1], coeff->values[2]);
        Eigen::Vector4f c; pcl::compute3DCentroid(*roi, *inliers, c);
        if (n.dot(c.head<3>()) > 0) n = -n;

        LaserLine l;
        l.label = reg.label;
        l.avg_distance = std::abs(coeff->values[3]);
        l.normal = n;
        l.confidence = inliers->indices.size();
        for(int idx : inliers->indices) l.raw_points.push_back(roi->points[idx]);

        if (reg.label == "left") { left = l; has_l = true; }
        else if (reg.label == "right") { right = l; has_r = true; }
        else if (reg.label == "front") { front = l; has_f = true; }
    }
    
    return has_l && has_r && has_f;
}

void LidarCalibration::printFinalResult(const Eigen::Matrix4f& T)
{
    Eigen::Vector3f t = T.block<3,1>(0,3);
    Eigen::Vector3f e = T.block<3,3>(0,0).eulerAngles(2,1,0); 
    
    std::cout << "\033[2J\033[1;1H"; 
    std::cout << "============================================" << std::endl;
    std::cout << "        EXTRINSIC CALIBRATION RESULT        " << std::endl;
    std::cout << "      Child: Mid  -->  Parent: Velodyne     " << std::endl;
    std::cout << "      Status: Averaging " << calibration_queue_.size() << " / " << SMOOTH_WINDOW_SIZE << " frames" << std::endl;
    std::cout << "============================================" << std::endl;
    std::cout << std::fixed << std::setprecision(4);
    
    std::cout << "Translation (m):" << std::endl;
    std::cout << "  x: " << t.x() << std::endl;
    std::cout << "  y: " << t.y() << std::endl;
    std::cout << "  z: " << t.z() << std::endl;
    std::cout << std::endl;
    
    std::cout << "Rotation (Euler RPY in radians):" << std::endl;
    std::cout << "  roll:  " << e[2] << std::endl;
    std::cout << "  pitch: " << e[1] << std::endl;
    std::cout << "  yaw:   " << e[0] << std::endl;
    std::cout << std::endl;
    
    std::cout << "Rotation (Euler RPY in degrees):" << std::endl;
    std::cout << "  roll:  " << toDegrees(e[2]) << " deg" << std::endl;
    std::cout << "  pitch: " << toDegrees(e[1]) << " deg" << std::endl;
    std::cout << "  yaw:   " << toDegrees(e[0]) << " deg" << std::endl;
    std::cout << "============================================" << std::endl;
}

// [新增] 将标定结果保存为 YAML 格式
void LidarCalibration::saveToYAML(const Eigen::Matrix4f& T, const std::string& filename)
{
    std::ofstream out(filename);
    if (!out.is_open()) {
        ROS_ERROR_THROTTLE(1.0, "无法打开文件以保存标定结果: %s", filename.c_str());
        return;
    }

    Eigen::Vector3f t = T.block<3,1>(0,3);
    // eulerAngles(2,1,0) 返回的是 [yaw, pitch, roll]
    Eigen::Vector3f e = T.block<3,3>(0,0).eulerAngles(2,1,0); 
    Eigen::Quaternionf q(T.block<3,3>(0,0));

    out << std::fixed << std::setprecision(6);
    out << "lidar_extrinsics:\n";
    out << "  parent_frame: \"" << lidar_frame_ << "\"\n";
    out << "  child_frame: \"" << mid_frame_ << "\"\n";
    
    out << "  translation: [x, y, z]\n";
    out << "    x: " << t.x() << "\n";
    out << "    y: " << t.y() << "\n";
    out << "    z: " << t.z() << "\n";
    
    out << "  rotation_euler_rad: [roll, pitch, yaw]\n";
    out << "    roll: " << e[2] << "\n";
    out << "    pitch: " << e[1] << "\n";
    out << "    yaw: " << e[0] << "\n";

    out << "  rotation_euler_deg: [roll, pitch, yaw]\n";
    out << "    roll: " << toDegrees(e[2]) << "\n";
    out << "    pitch: " << toDegrees(e[1]) << "\n";
    out << "    yaw: " << toDegrees(e[0]) << "\n";
    
    out << "  rotation_quaternion: [x, y, z, w]\n";
    out << "    x: " << q.x() << "\n";
    out << "    y: " << q.y() << "\n";
    out << "    z: " << q.z() << "\n";
    out << "    w: " << q.w() << "\n";
    
    out << "  transformation_matrix:\n";
    for (int i = 0; i < 4; ++i) {
        out << "    - [" << T(i,0) << ", " << T(i,1) << ", " << T(i,2) << ", " << T(i,3) << "]\n";
    }

    out.close();
}

float LidarCalibration::toRadians(float d) { return d * M_PI / 180.0f; }
float LidarCalibration::toDegrees(float r) { return r * 180.0f / M_PI; }

void LidarCalibration::run() { ros::spin(); }

