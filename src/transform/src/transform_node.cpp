#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/buffer.h"
#include <tf2/transform_datatypes.h>
#include <tf2/utils.h>
#include "tf2_sensor_msgs/tf2_sensor_msgs.hpp" //for tf transform
#include "tf2_ros/transform_listener.h"

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>



class Tf_Points: public rclcpp::Node{
public:
    Tf_Points():Node("transform_node"){
        RCLCPP_INFO(this->get_logger(),"订阅点云...");

        broadcaster_= std::make_shared<tf2_ros::TransformBroadcaster>(this);
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        pc_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>("lslidar_point_cloud", 10, std::bind(&Tf_Points::do_cb, this, std::placeholders::_1));
        pc_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("tf_pointcloud2", 10);

        this->declare_parameter<bool>("filter_pass_x", false);   
        this->declare_parameter<bool>("filter_pass_y", false);   
        this->declare_parameter<bool>("filter_pass_z", false);   
        this->declare_parameter<double>("filter_pass_x_min", -100.0);   
        this->declare_parameter<double>("filter_pass_x_max", 100.0);   
        this->declare_parameter<double>("filter_pass_y_min", -100.0);   
        this->declare_parameter<double>("filter_pass_y_max", 100.0);   
        this->declare_parameter<double>("filter_pass_z_min", -100.0);   
        this->declare_parameter<double>("filter_pass_z_max", 100.0);  
        this->declare_parameter<bool>("filter_removal", false);   
        this->declare_parameter<int>("meank", 50);   
        this->declare_parameter<int>("threshold", 1);   
        this->declare_parameter<bool>("filter_voxel", false);  
        this->declare_parameter<double>("size_x", 0.1);  
        this->declare_parameter<double>("size_y", 0.1);  
        this->declare_parameter<double>("size_z", 0.1);  
        this->declare_parameter<bool>("tf", false);  
        this->declare_parameter<double>("tf_x", 0.0);  
        this->declare_parameter<double>("tf_y", 0.0);  
        this->declare_parameter<double>("tf_z", 0.0);  
        this->declare_parameter<double>("tf_roll", 0.0);  
        this->declare_parameter<double>("tf_pitch", 0.0);  
        this->declare_parameter<double>("tf_yaw", 0.0);  

        this->get_parameter<bool>("filter_pass_x", filter_pass_x);
        this->get_parameter<bool>("filter_pass_y", filter_pass_y);
        this->get_parameter<bool>("filter_pass_z", filter_pass_z);
        this->get_parameter<double>("filter_pass_x_min", filter_pass_x_min);
        this->get_parameter<double>("filter_pass_x_max", filter_pass_x_max);
        this->get_parameter<double>("filter_pass_y_min", filter_pass_y_min);
        this->get_parameter<double>("filter_pass_y_max", filter_pass_y_max);
        this->get_parameter<double>("filter_pass_z_min", filter_pass_z_min);
        this->get_parameter<double>("filter_pass_z_max", filter_pass_z_max);
        this->get_parameter<bool>("filter_removal", filter_removal);
        this->get_parameter<int>("meank", meank);
        this->get_parameter<int>("threshold", threshold);
        this->get_parameter<bool>("filter_voxel", filter_voxel);
        this->get_parameter<double>("size_x", size_x);
        this->get_parameter<double>("size_y", size_y);
        this->get_parameter<double>("size_z", size_z);
        this->get_parameter<bool>("tf", tf);
        this->get_parameter<double>("tf_x", tf_x);
        this->get_parameter<double>("tf_y", tf_y);
        this->get_parameter<double>("tf_z", tf_z);
        this->get_parameter<double>("tf_roll", tf_roll);
        this->get_parameter<double>("tf_pitch", tf_pitch);
        this->get_parameter<double>("tf_yaw", tf_yaw);
    }

private:
    bool filter_pass_x,filter_pass_y,filter_pass_z,filter_removal,filter_voxel,tf;
    double filter_pass_x_min,filter_pass_x_max,filter_pass_y_min,filter_pass_y_max,filter_pass_z_min,filter_pass_z_max,
            size_x,size_y,size_z,tf_x,tf_y,tf_z,tf_roll,tf_pitch,tf_yaw;
    int meank,threshold;
    
    pcl::PCLPointCloud2 cloud;
    pcl::PassThrough<pcl::PCLPointCloud2> pass;
    pcl::StatisticalOutlierRemoval<pcl::PCLPointCloud2> sor_removal;
    pcl::VoxelGrid<pcl::PCLPointCloud2> sor_voxel;
    sensor_msgs::msg::PointCloud2 cloud_done_ros2;
    geometry_msgs::msg::TransformStamped transformStamped;
    geometry_msgs::msg::TransformStamped ts;

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pc_sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pc_pub_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> broadcaster_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

    void do_cb(const sensor_msgs::msg::PointCloud2 msg){
        // RCLCPP_INFO(this->get_logger(), "Received point cloud with %lu points", msg.data.size());
        pcl_conversions::toPCL(msg, cloud);
        //转换ROS2->PCLPointcloud2，并创建 cloud_pcl指针指向它
        pcl::PCLPointCloud2::Ptr cloud_done_ptr(new pcl::PCLPointCloud2(cloud));

        if (filter_pass_x)
        {
            //直通滤波
            // RCLCPP_WARN(this->get_logger(), "直通滤波 X-min: %.2f",filter_pass_x_min);
            // RCLCPP_WARN(this->get_logger(), "直通滤波 X-max: %.2f",filter_pass_x_max);
            pass.setInputCloud(cloud_done_ptr);
            pass.setFilterFieldName("x");
            pass.setFilterLimits(filter_pass_x_min, filter_pass_x_max);
            pass.filter(*cloud_done_ptr);
        }
        if (filter_pass_y)
        {
            //直通滤波
            // RCLCPP_WARN(this->get_logger(), "直通滤波 Y-min: %.2f",filter_pass_y_min);
            // RCLCPP_WARN(this->get_logger(), "直通滤波 Y-max: %.2f",filter_pass_y_max);
            pass.setInputCloud(cloud_done_ptr);
            pass.setFilterFieldName("y");
            pass.setFilterLimits(filter_pass_y_min, filter_pass_y_max);
            pass.filter(*cloud_done_ptr);
        }
        if (filter_pass_z)
        {
            //直通滤波
            // RCLCPP_WARN(this->get_logger(), "Z直通滤波 Z-min: %.2f",filter_pass_z_min);
            // RCLCPP_WARN(this->get_logger(), "Z直通滤波 Z-max: %.2f",filter_pass_z_max);
            pass.setInputCloud(cloud_done_ptr);
            pass.setFilterFieldName("z");
            pass.setFilterLimits(filter_pass_z_min, filter_pass_z_max);
            pass.filter(*cloud_done_ptr);
        }
        
        if (filter_removal)
        {
            //离群滤波
            // RCLCPP_WARN(this->get_logger(), "离群滤波 MeanK: %i", meank);
            sor_removal.setInputCloud(cloud_done_ptr);
            sor_removal.setMeanK(meank);   //统计时考虑查询的临近点数
            sor_removal.setStddevMulThresh(threshold);  //离群阈值，点的距离超出平均距离一个标准差
            sor_removal.filter(*cloud_done_ptr);
        }

        if (filter_voxel)
        {
            //体素滤波
            // RCLCPP_WARN(this->get_logger(), "体素滤波 Voxel: %.2f", size_x);
            sor_voxel.setInputCloud(cloud_done_ptr);
            sor_voxel.setLeafSize(size_x, size_y, size_z);
            sor_voxel.filter(*cloud_done_ptr);
        }
        //转换为ROS2 
        pcl_conversions::fromPCL(*cloud_done_ptr, cloud_done_ros2);

        if (tf)
        {
            //tf变换
            send_tf();
            // RCLCPP_WARN(this->get_logger(), "坐标变换");
            auto out = std::make_shared<sensor_msgs::msg::PointCloud2>(); 
            try {
                transformStamped = tf_buffer_->lookupTransform(
                    "crane", "laser_link", tf2::TimePointZero); 
            } catch (tf2::TransformException &ex) {
                RCLCPP_WARN(this->get_logger(),"tf:%s", ex.what());
                // SPDLOG_ERROR("{}", ex.what());
                return;
            }
            tf2::doTransform(cloud_done_ros2, *out, transformStamped);
            //发布
            pc_pub_->publish(*out);
            // pc_pub_->publish(cloud_voxe_ros2);
            // RCLCPP_INFO(this->get_logger(), "Send point cloud with %lu points", out->data.size());

        }     
    }
    void send_tf(){
        // RCLCPP_INFO(this->get_logger(),"发布坐标 Roll: %.2f", tf_roll);
        
        ts.header.stamp = this->now();
        ts.header.frame_id = "crane";
        ts.child_frame_id = "laser_link";
        ts.transform.translation.x = tf_x;
        ts.transform.translation.y = tf_y;
        ts.transform.translation.z = tf_z;

        tf2::Quaternion qtn;
        qtn.setRPY(tf_roll, tf_pitch, tf_yaw);

        ts.transform.rotation.x = qtn.x();
        ts.transform.rotation.y = qtn.y();
        ts.transform.rotation.z = qtn.z();
        ts.transform.rotation.w = qtn.w();

        broadcaster_->sendTransform(ts);
    }
};

int main(int argc, char const *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Tf_Points>());
    rclcpp::shutdown();
    return 0;
}