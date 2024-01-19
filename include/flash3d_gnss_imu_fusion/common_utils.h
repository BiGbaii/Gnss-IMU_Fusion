#pragma once
#include<eigen3/Eigen/Core>
#include<LocalCartesian.hpp>
#include<memory>
#include<rclcpp/rclcpp.hpp>

namespace Flash3dGnssImuFison
{

using V3D = Eigen::Vector3d;
using M3D =Eigen::Matrix3d;

struct IMU
{
    double time_stamp;
    V3D acc;
    V3D gyro;
};
using IMU_ptr = std::shared_ptr<IMU>;
 
struct GPS
{
    double time_stamp;
    //latitude in degree, longitude in degree, and altitude in meter.
    //WGS84
    V3D lla; 
    //ENU frame
    V3D enu;
    // Covariance in m^2.
    M3D cov;
};
using GPS_ptr = std::shared_ptr<GPS>;

inline void convert_LLA2ENU(const V3D&  inital_lla,GPS_ptr GPS_data)
{
    static GeographicLib::LocalCartesian geo_converter;
    geo_converter.Reset(inital_lla(0),inital_lla(1),inital_lla(2));
    geo_converter.Forward(GPS_data->lla(0),GPS_data->lla(1),GPS_data->lla(2),
                            GPS_data->enu(0),GPS_data->enu(1),GPS_data->enu(2));
}

inline void convert_ENU2LLA(const V3D&  inital_lla,GPS_ptr GPS_data)
{
    static GeographicLib::LocalCartesian geo_converter;
    geo_converter.Reset(inital_lla(0),inital_lla(1),inital_lla(2));
    geo_converter.Reverse(GPS_data->enu(0),GPS_data->enu(1),GPS_data->enu(2),
                            GPS_data->lla(0),GPS_data->lla(1),GPS_data->lla(2));
}

inline M3D skew_matrix3d(const V3D& v)
{
    M3D skew_m;
    skew_m<< 0.0 , -v(2) , v(1),
             v(2) , 0.0 , -v(0),
             -v(1) , v(0) ,0.0;

    return skew_m;
}

inline double get_time_sec(const builtin_interfaces::msg::Time &time)
{
    return rclcpp::Time(time).seconds();
}

}
