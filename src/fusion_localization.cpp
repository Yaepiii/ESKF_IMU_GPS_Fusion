#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <Eigen/Dense>
#include <memory>
#include <functional>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include "tf2_ros/transform_broadcaster.h"
#include <deque>
#include <algorithm>
#include <fstream>
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include "imu_gps_localizer/base_type.h"
#include "imu_gps_localizer/imu_gps_localizer.h"
#include "imu_gps_localizer/utils.h"

class FusionLocalization : public rclcpp::Node
{
public:

    FusionLocalization() : Node("odom_reloc_subscriber")
    {
        // 初始化path消息
        fusion_path_msg_.header.frame_id = "map";
        fusion_path_msg_.header.stamp = this->now();
        gps_path_msg_.header.frame_id = "map";
        gps_path_msg_.header.stamp = this->now();

        // 声明参数并设置默认值
        this->declare_parameter<std::string>("log_root", "/home/lyx/");
        this->declare_parameter<std::string>("imu_topic", "/imu_data");
        this->declare_parameter<std::string>("gps_topic", "/gps");
        this->declare_parameter<double>("acc_noise", 0.01);
        this->declare_parameter<double>("gyro_noise", 0.01);
        this->declare_parameter<double>("acc_bias_noise", 0.0001);
        this->declare_parameter<double>("gyro_bias_noise", 0.0001);
        this->declare_parameter<double>("I_p_Gps_x", 0.0);
        this->declare_parameter<double>("I_p_Gps_y", 0.0);
        this->declare_parameter<double>("I_p_Gps_z", 0.0);

        // 获取参数值
        this->get_parameter("log_root", log_root_);
        this->get_parameter("imu_topic", imu_topic_);
        this->get_parameter("gps_topic", gps_topic_);
        this->get_parameter("acc_noise", acc_noise_);
        this->get_parameter("gyro_noise", gyro_noise_);
        this->get_parameter("acc_bias_noise", acc_bias_noise_);
        this->get_parameter("gyro_bias_noise", gyro_bias_noise_);
        double I_p_Gps_x_, I_p_Gps_y_, I_p_Gps_z_;
        this->get_parameter("I_p_Gps_x", I_p_Gps_x_);
        this->get_parameter("I_p_Gps_y", I_p_Gps_y_);
        this->get_parameter("I_p_Gps_z", I_p_Gps_z_);
        I_p_Gps_(0) = I_p_Gps_x_;
        I_p_Gps_(1) = I_p_Gps_y_;
        I_p_Gps_(2) = I_p_Gps_z_;
        
        // 打印检查
        RCLCPP_INFO(this->get_logger(), "[Params] log_root is: %s\n", log_root_.c_str());
        RCLCPP_INFO(this->get_logger(), "[Params] imu_topic is: %s\n", imu_topic_.c_str());
        RCLCPP_INFO(this->get_logger(), "[Params] gps_topic is: %s\n", gps_topic_.c_str());
        RCLCPP_INFO(this->get_logger(), "[Params] acc_noise is: %f\n", acc_noise_);
        RCLCPP_INFO(this->get_logger(), "[Params] gyro_noise is: %f\n", gyro_noise_);
        RCLCPP_INFO(this->get_logger(), "[Params] acc_bias_noise is: %f\n", acc_bias_noise_);
        RCLCPP_INFO(this->get_logger(), "[Params] gyro_bias_noise is: %f\n", gyro_bias_noise_);
        RCLCPP_INFO(this->get_logger(), "[Params] I_p_Gps_x is: %f\n", I_p_Gps_x_);
        RCLCPP_INFO(this->get_logger(), "[Params] I_p_Gps_y is: %f\n", I_p_Gps_y_);
        RCLCPP_INFO(this->get_logger(), "[Params] I_p_Gps_z is: %f\n", I_p_Gps_z_);

        // Initialization eskf localizer.
        imu_gps_localizer_ptr_ = 
            std::make_unique<ImuGpsLocalization::ImuGpsLocalizer>(acc_noise_, gyro_noise_,
                                                              acc_bias_noise_, gyro_bias_noise_,
                                                              I_p_Gps_);

        // 创建订阅者 
        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            imu_topic_, 1,
            std::bind(&FusionLocalization::imu_callback, this, std::placeholders::_1));
        gps_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
            gps_topic_, 1,
            std::bind(&FusionLocalization::gps_callback, this, std::placeholders::_1));

        // 创建发布者
        fusion_path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/fusion_path", 1);
        gps_path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/gps_path", 1);
        fusion_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/localization_fusion", 1);

        // 初始化TF广播器
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        // 打开位姿保存文件
        fusion_pose_.open(log_root_+"fusion_pose.txt", std::fstream::out);
		if(!fusion_pose_.is_open())
		{
			std::cerr << "fusion_pose_ is not open..........." << std::endl;
		}
		else
		{
			std::cerr << "fusion_pose_ file open successful ..........." << std::endl;
		}

        // 打开位姿保存文件
        gps_pose_.open(log_root_+"gps_pose.txt", std::fstream::out);
		if(!fusion_pose_.is_open())
		{
			std::cerr << "gps_pose_ is not open..........." << std::endl;
		}
		else
		{
			std::cerr << "gps_pose_ file open successful ..........." << std::endl;
		}
    }

    ~FusionLocalization()
    {
        fusion_pose_.close();
        gps_pose_.close();
        gps_pose_.close();
    }

private:

    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        ImuGpsLocalization::ImuDataPtr imu_data_ptr = std::make_shared<ImuGpsLocalization::ImuData>();
        rclcpp::Time imu_time = rclcpp::Time(msg->header.stamp);
        imu_data_ptr->timestamp = imu_time.seconds();
        imu_data_ptr->acc << msg->linear_acceleration.x, 
                            msg->linear_acceleration.y,
                            msg->linear_acceleration.z;
        imu_data_ptr->gyro << msg->angular_velocity.x,
                            msg->angular_velocity.y,
                            msg->angular_velocity.z;
        
        ImuGpsLocalization::State fused_state;
        const bool ok = imu_gps_localizer_ptr_->ProcessImuData(imu_data_ptr, &fused_state);
        if (!ok) {
            return;
        }

        // Publish fused state.
        ConvertStateToRosTopic(fused_state);

        // Log fused state.
        LogState(fused_state);
    }

    void gps_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
    {
        // Check the gps_status.
        // if (msg->status.status != 2) {
        //     return;
        // }

        ImuGpsLocalization::GpsPositionDataPtr gps_data_ptr = std::make_shared<ImuGpsLocalization::GpsPositionData>();
        rclcpp::Time gps_time = rclcpp::Time(msg->header.stamp);
        gps_data_ptr->timestamp = gps_time.seconds();
        gps_data_ptr->lla << msg->latitude,
                            msg->longitude,
                            msg->altitude;
        gps_data_ptr->cov = Eigen::Map<const Eigen::Matrix3d>(msg->position_covariance.data());

        imu_gps_localizer_ptr_->ProcessGpsPositionData(gps_data_ptr);

        LogGps(gps_data_ptr);
    }

    void ConvertStateToRosTopic(const ImuGpsLocalization::State& state) {
        fusion_path_msg_.header.stamp = this->now();
        geometry_msgs::msg::PoseStamped pose_stamped;
        pose_stamped.header = fusion_path_msg_.header;

        pose_stamped.pose.position.x = state.G_p_I[0];
        pose_stamped.pose.position.y = state.G_p_I[1];
        pose_stamped.pose.position.z = state.G_p_I[2];

        const Eigen::Quaterniond G_q_I(state.G_R_I);
        pose_stamped.pose.orientation.x = G_q_I.x();
        pose_stamped.pose.orientation.y = G_q_I.y();
        pose_stamped.pose.orientation.z = G_q_I.z();
        pose_stamped.pose.orientation.w = G_q_I.w();

        fusion_path_msg_.poses.push_back(pose_stamped);
        fusion_path_pub_->publish(fusion_path_msg_);

        // 发布localization_fusion话题 - 使用预测状态
        geometry_msgs::msg::PoseStamped fusion_pose;
        fusion_pose.header = fusion_path_msg_.header;
        fusion_pose.header.frame_id = "map";
        fusion_pose.pose.position.x = state.G_p_I[0];
        fusion_pose.pose.position.y = state.G_p_I[1];
        fusion_pose.pose.position.z = state.G_p_I[2];
        fusion_pose.pose.orientation.x = G_q_I.x();
        fusion_pose.pose.orientation.y = G_q_I.y();
        fusion_pose.pose.orientation.z = G_q_I.z();
        fusion_pose.pose.orientation.w = G_q_I.w();
        fusion_pose_pub_->publish(fusion_pose);

        // 发布map到fusion的TF变换 - 使用预测状态
        geometry_msgs::msg::TransformStamped t;
        t.header = fusion_path_msg_.header;
        t.header.frame_id = "map";
        t.child_frame_id = "fusion";
        t.transform.translation.x = state.G_p_I[0];
        t.transform.translation.y = state.G_p_I[1];
        t.transform.translation.z = state.G_p_I[2];
        t.transform.rotation = fusion_pose.pose.orientation;
        tf_broadcaster_->sendTransform(t);
    }

    void LogState(const ImuGpsLocalization::State& state) {
        static double first_time = 0;
        if (first_time == 0) {
            first_time = state.timestamp;
        }
        const Eigen::Quaterniond G_q_I(state.G_R_I);
        fusion_pose_ << std::fixed << std::setprecision(15)
                    << (state.timestamp - first_time) << " "
                    << state.G_p_I[0] << " " << state.G_p_I[1] << " " << state.G_p_I[2] << " "
                    << G_q_I.x() << " " << G_q_I.y() << " " << G_q_I.z() << " " << G_q_I.w() << "\n";
    }

    void LogGps(const ImuGpsLocalization::GpsPositionDataPtr gps_data) {
        static double first_time = 0;
        static Eigen::Vector3d init_lla_local;
        if (first_time == 0) {
            init_lla_local = gps_data->lla;
            first_time = gps_data->timestamp;
        }
        Eigen::Vector3d G_p_Gps;
        ImuGpsLocalization::LLAToENU(init_lla_local, gps_data->lla, &G_p_Gps);
        gps_pose_ << std::fixed << std::setprecision(15)
                << (gps_data->timestamp - first_time) << " "
                << G_p_Gps(0) << " " << G_p_Gps(1) << " " << G_p_Gps(2) << " "
                << 0.0 << " " << 0.0 << " " << 0.0 << " " << 1.0 << "\n";
        gps_path_msg_.header.stamp = this->now();
        geometry_msgs::msg::PoseStamped pose_stamped;
        pose_stamped.header = gps_path_msg_.header;

        pose_stamped.pose.position.x = G_p_Gps(0);
        pose_stamped.pose.position.y = G_p_Gps(1);
        pose_stamped.pose.position.z = G_p_Gps(2);
        pose_stamped.pose.orientation.x = 0;
        pose_stamped.pose.orientation.y = 0;
        pose_stamped.pose.orientation.z = 0;
        pose_stamped.pose.orientation.w = 1;

        gps_path_msg_.poses.push_back(pose_stamped);
        gps_path_pub_->publish(gps_path_msg_);
    }

    // 话题信息
    std::string imu_topic_;
    std::string gps_topic_;

    // IMU相关变量
    double acc_noise_, gyro_noise_, acc_bias_noise_, gyro_bias_noise_;
    Eigen::Vector3d I_p_Gps_;

    // 订阅者和发布者
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr fusion_path_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr gps_path_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr fusion_pose_pub_;

    // Path消息
    nav_msgs::msg::Path fusion_path_msg_;
    nav_msgs::msg::Path gps_path_msg_;

    // TF广播器
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    // GINS状态估计器
    std::unique_ptr<ImuGpsLocalization::ImuGpsLocalizer> imu_gps_localizer_ptr_;

    // 位姿估计结果输出文件
    std::string log_root_;
    std::ofstream fusion_pose_;
    std::ofstream gps_pose_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto fl_node = std::make_shared<FusionLocalization>();
    rclcpp::spin(fl_node);
    rclcpp::shutdown();
    return 0;
}
