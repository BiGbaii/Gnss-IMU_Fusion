#include"flash3d_gnss_imu_fusion/gins.h"
#include<glog/logging.h>


Flash3dGnssImuFison::GNSS_IMU_ESKF GinsNode::state;

void GinsNode::imu_callback(const sensor_msgs::msg::Imu::UniquePtr imu_in)
{

    Flash3dGnssImuFison::IMU_ptr imu_data = std::make_shared<Flash3dGnssImuFison::IMU>();
    imu_data->acc << imu_in->linear_acceleration.x,
                     imu_in->linear_acceleration.y,
                     imu_in->linear_acceleration.z;
    imu_data->gyro << imu_in->angular_velocity.x,
                      imu_in->angular_velocity.y,
                      imu_in->angular_velocity.z;
    imu_data->time_stamp = Flash3dGnssImuFison::get_time_sec(imu_in->header.stamp);

    if(!this->imu_static_initaler.is_initalized())
    {
        imu_static_initaler.add_imu_data(imu_data);
        return;
    }
    
    this->state.predict(imu_data);

    //when recive imu ,publish path to get more frequency,
    //instead of publish path when recive gps;
    convert_ros2_topic(this->state);
    
    pub_path->publish(ros_path);

}

void GinsNode::gps_callback(const sensor_msgs::msg::NavSatFix::UniquePtr gps_in)
{

    if(gps_in->status.status == sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX)
    {
        LOG(WARNING)<<"[GPS_CALLBACK]: bad GPS message!";
        return;
    }
    

    Flash3dGnssImuFison::GPS_ptr gps_data =  std::make_shared<Flash3dGnssImuFison::GPS>();
    gps_data->lla << gps_in->latitude,
                     gps_in->longitude,
                     gps_in->altitude;


    gps_data->time_stamp = Flash3dGnssImuFison::get_time_sec(gps_in->header.stamp);


    gps_data->cov = Eigen::Map<const Flash3dGnssImuFison::M3D>(gps_in->position_covariance.data());

    if(!this->imu_static_initaler.is_initalized())
    {
        inital_lla_ = gps_data->lla;
        Flash3dGnssImuFison::convert_LLA2ENU(inital_lla_,gps_data);
        if(!imu_static_initaler.static_initalize(gps_data,state)) return;
        LOG(INFO) << "[gps_callback]: System initialized!";
        return;
    }

    state.observe_gps(inital_lla_,gps_data);

}


void GinsNode::convert_ros2_topic(const Flash3dGnssImuFison::GNSS_IMU_ESKF & state)
{
    ros_path.header.frame_id = "flash3d_world";
    ros_path.header.stamp = rclcpp::Clock().now();

    geometry_msgs::msg::PoseStamped pose_stamped;
    pose_stamped.header=ros_path.header;
    pose_stamped.pose.position.x = state.p_[0];
    pose_stamped.pose.position.y = state.p_[1];
    pose_stamped.pose.position.z = state.p_[2];

    Eigen::Quaterniond quaternion(state.R_);
    pose_stamped.pose.orientation.x = quaternion.x();
    pose_stamped.pose.orientation.y = quaternion.y();
    pose_stamped.pose.orientation.z = quaternion.z();
    pose_stamped.pose.orientation.w = quaternion.w();

    ros_path.poses.push_back(pose_stamped);
}