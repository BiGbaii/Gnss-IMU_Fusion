#pragma once
#include<rclcpp/rclcpp.hpp>
#include<sensor_msgs/msg/imu.hpp>
#include<sensor_msgs/msg/nav_sat_fix.hpp>
#include"flash3d_gnss_imu_fusion/flash3d_eskf.h"
#include<Eigen/Core>
#include<Eigen/Dense>

#include<nav_msgs/msg/path.hpp>
#include<geometry_msgs/msg/pose_stamped.hpp>


class GinsNode :public rclcpp::Node
{
public:
    GinsNode(const std::string& name):Node(name)
    {
        RCLCPP_INFO(this->get_logger(),"INIT GINS_Node!");
        this->declare_parameter<double>("acc_bias_std",0.0001);
        this->declare_parameter<double>("gyro_bias_std",0.000001);
        this->declare_parameter<std::string>("imu_topic","/imu/data");
        this->declare_parameter<std::string>("gps_topic","/fix");
        this->declare_parameter<double>("I_p_Gps_x",0.0);
        this->declare_parameter<double>("I_p_Gps_y",0.0);
        this->declare_parameter<double>("I_p_Gps_z",0.0);

        this->get_parameter<double>("acc_bias_std",acc_bias_std);
        this->get_parameter<double>("gyro_bias_std",gyro_bias_std);
        this->get_parameter<std::string>("imu_topic",imu_topic);
        this->get_parameter<std::string>("gps_topic",gps_topic);
        double x,y,z;
        this->get_parameter<double>("I_p_Gps_x",x);
        this->get_parameter<double>("I_p_Gps_y",y);
        this->get_parameter<double>("I_p_Gps_z",z);
        const Flash3dGnssImuFison::V3D I_p_Gps(x,y,z);
        state.I_p_Gps_ = I_p_Gps;

        sub_imu = this->create_subscription<sensor_msgs::msg::Imu>(imu_topic,20,std::bind(&GinsNode::imu_callback,this,std::placeholders::_1));
        sub_gps = this->create_subscription<sensor_msgs::msg::NavSatFix>(gps_topic,20,std::bind(&GinsNode::gps_callback,this,std::placeholders::_1));
        pub_path = this->create_publisher<nav_msgs::msg::Path>("/flash3d_GINS_path",20);

        //TODO//
        RCLCPP_INFO(this->get_logger(),"GINS_Node init successful!");
    }
    
private:
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr sub_gps;

    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_path;
    /*************/
    double acc_bias_std=0.0;
    double gyro_bias_std=0.0;
    std::string imu_topic;
    std::string gps_topic;
    static Flash3dGnssImuFison::GNSS_IMU_ESKF state;
    Flash3dGnssImuFison::Initalizer imu_static_initaler;
    Flash3dGnssImuFison::V3D inital_lla_;

    nav_msgs::msg::Path ros_path;
private:
    void imu_callback(const sensor_msgs::msg::Imu::UniquePtr imu_in);
    void gps_callback(const sensor_msgs::msg::NavSatFix::UniquePtr gps_in);
    //when imu comes ,publish path
    void convert_ros2_topic(const Flash3dGnssImuFison::GNSS_IMU_ESKF & state);//考虑explicit 的构造函数
};
