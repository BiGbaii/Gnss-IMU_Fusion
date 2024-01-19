#pragma once
#include"flash3d_gnss_imu_fusion/common_utils.h"
#include<deque>

namespace Flash3dGnssImuFison
{

constexpr double kDegreeToRadian = M_PI / 180.;
constexpr double kRadianToDegree = 180. / M_PI;

class GNSS_IMU_ESKF
{
public:
    void predict(const IMU_ptr cur_imu);
    //update
    void observe_gps(const V3D& inital_lla, GPS_ptr gps_data);

public:
    //time stamp
    double time_stamp=0.0;
    IMU_ptr last_imu;


    //nominal-state
    V3D p_;
    V3D v_;
    M3D R_;
    V3D acc_bias;
    V3D gyro_bias;
    V3D g_{0,0,-9.81};
    
    //covariance
    Eigen::Matrix<double,18,18> cov_;

    // The following  values need to be initialized
    //motion noise
    double acc_noise_=1e-2;
    double gyro_noise_=1e-4;
    double acc_bias_noise_=1e-6;
    double gyro_bias_noise_=1e-8;

    //Eigen::Matrix<double,18,18> Q;
    V3D I_p_Gps_;  


private:
    void compute_jacobin_residual(const V3D & inital_lla,GPS_ptr gps_data, \
                                  Eigen::Matrix<double,3,18>* H,
                                  V3D *residual);

    void update_deltaX_and_reset(const Eigen::Matrix<double, 18, 1>& delta_x);

};

class Initalizer
{
public:
    void add_imu_data(const IMU_ptr imu_data);
    //when gnss data come need to keep gnss&imu sync
    bool static_initalize(const GPS_ptr gps_data, GNSS_IMU_ESKF & state);
    bool is_initalized();

private:

    std::deque<IMU_ptr> imu_buffer;
    bool imu_inital_flag=false;
    V3D acc_std;
    V3D gyro_std;
};


}