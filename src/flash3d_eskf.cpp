#include"flash3d_gnss_imu_fusion/flash3d_eskf.h"
#include<eigen3/Eigen/Dense>
#include<glog/logging.h>


void Flash3dGnssImuFison::GNSS_IMU_ESKF::predict(const IMU_ptr cur_imu)
{
    double dt = cur_imu->time_stamp - time_stamp;
    if(dt<=0)
    {
        LOG(INFO) << "[predict]: skip this imu because dt = " <<dt;
        //printf("skip this imu because dt = %d",dt);
        return;
    }


    //nominal state
    p_=p_+v_*dt +0.5*(R_*(cur_imu->acc-acc_bias))*dt*dt+0.5*g_*dt*dt;
    v_=v_+R_*(cur_imu->acc-acc_bias)*dt+g_*dt;
    const V3D delta_angle_axis = (cur_imu->gyro-gyro_bias) * dt;
    R_=R_*Eigen::AngleAxisd(delta_angle_axis.norm(), delta_angle_axis.normalized()).toRotationMatrix();

    //error state
    //
    Eigen::Matrix<double,18,18> Fx = Eigen::Matrix<double,18,18>::Identity();
    Fx.block<3,3>(0,3)=M3D::Identity() * dt;
    Fx.block<3,3>(3,6)= -R_ * skew_matrix3d(cur_imu->acc - acc_bias)*dt;
    Fx.block<3,3>(3,9)= -R_*dt;
    Fx.block<3,3>(3,15)=M3D::Identity()*dt;
    Fx.block<3,3>(6,6)=Eigen::AngleAxisd(delta_angle_axis.norm(),delta_angle_axis.normalized()).toRotationMatrix().transpose();
    Fx.block<3,3>(6,12)= -M3D::Identity()*dt;

    //covariance
    Eigen::Matrix<double,18,12> Fi =Eigen::Matrix<double,18,12>::Zero();
    Fi.block<12,12>(3,0)=Eigen::Matrix<double,12,12>::Identity();

    Eigen::Matrix<double, 12, 12> Qi = Eigen::Matrix<double, 12, 12>::Zero();
    Qi.block<3, 3>(0, 0) = dt * acc_noise_ * Eigen::Matrix3d::Identity();
    Qi.block<3, 3>(3, 3) = dt * gyro_noise_ * Eigen::Matrix3d::Identity();
    Qi.block<3, 3>(6, 6) = dt*dt * acc_bias_noise_ * Eigen::Matrix3d::Identity();
    Qi.block<3, 3>(9, 9) = dt*dt * gyro_bias_noise_ * Eigen::Matrix3d::Identity();

    cov_ = Fx*cov_*Fx.transpose() + Fi* Qi *Fi.transpose();

    time_stamp=cur_imu->time_stamp;
    last_imu=cur_imu;

}


void Flash3dGnssImuFison::GNSS_IMU_ESKF::observe_gps(const V3D& inital_lla, GPS_ptr gps_data)
{
    Eigen::Matrix<double,3,18> H;
    V3D residual;
    compute_jacobin_residual(inital_lla,gps_data,&H,&residual);

    const M3D &V = gps_data->cov;

    Eigen::Matrix<double,18,3> k = cov_ * H.transpose() * (H * cov_ *H.transpose() +V).inverse();
    Eigen::Matrix<double,18,1> dx= k* residual;
    Eigen::Matrix<double,18,18> I_kH = Eigen::Matrix<double,18,18>::Identity() - k * H;
    cov_ = I_kH*cov_*I_kH.transpose() + k*V*k.transpose();

    update_deltaX_and_reset(dx);


}


void Flash3dGnssImuFison::GNSS_IMU_ESKF::compute_jacobin_residual(const V3D & inital_lla,GPS_ptr gps_data, 
                                            Eigen::Matrix<double,3,18>* H,
                                            V3D *residual)
{

    const V3D G_p_I = p_;
    const M3D G_R_I = R_;

    convert_LLA2ENU(inital_lla,gps_data);

    //compute residual
    *residual=gps_data->enu - (G_p_I + (G_R_I * I_p_Gps_));

    //compute jacobin
    H->setZero();
    H->block<3,3>(0,0) = M3D::Identity();
    H->block<3,3>(0,6) = -G_R_I*skew_matrix3d(I_p_Gps_);
    
}

void Flash3dGnssImuFison::GNSS_IMU_ESKF::update_deltaX_and_reset(const Eigen::Matrix<double, 18, 1>& delta_x)
{
    this->p_ += delta_x.block<3,1>(0,0);
    this->v_ += delta_x.block<3,1>(3,0);
    this->R_ *= Eigen::AngleAxisd(delta_x.block<3, 1>(6, 0).norm(), delta_x.block<3, 1>(6, 0).normalized()).toRotationMatrix();
    this->acc_bias += delta_x.block<3,1>(9,0);
    this->gyro_bias += delta_x.block<3,1>(12,0);
    this->g_ += delta_x.block<3,1>(15,0);

    //reset projectCov
    //TODO//

}


void Flash3dGnssImuFison::Initalizer::add_imu_data(const IMU_ptr imu_data)
{
    imu_buffer.push_back(imu_data);
    if(imu_buffer.size() > 100)
    {
        imu_buffer.pop_front();
    }
}

bool Flash3dGnssImuFison::Initalizer::static_initalize(const GPS_ptr gps_data, GNSS_IMU_ESKF & state)
{
    if(imu_buffer.size()<100)
    {
        LOG(INFO) << "[static_initalize]:not enought imu data!";
        return false;
    }

    if(std::abs(gps_data->time_stamp - imu_buffer.back()->time_stamp) > 0.5 )
    {
        LOG(WARNING) << "[static_initalize]: gps time and IMU time are not sync!";
        return false;
    }

    V3D acc_sum{0.0,0.0,0.0};
    V3D gyr_sum{0.0,0.0,0.0};
    for(const auto & elem :imu_buffer)
    {
        acc_sum += elem->acc;
        gyr_sum += elem->gyro;
    }
    const V3D acc_mean = acc_sum / imu_buffer.size();
    const V3D gyr_mean = gyr_sum / imu_buffer.size();

    V3D sum_err{0.0,0.0,0.0};
    for(const auto & elem:imu_buffer)
    {
        sum_err += (elem->acc-acc_mean).cwiseAbs2();
    }
    const V3D err_std = (sum_err / (imu_buffer.size()-1)).cwiseSqrt();

    if(err_std.maxCoeff() >3)
    {
        LOG(WARNING) << "[static_inistalize]: too big acc_std!";
        return false;
    }

    // set timestamp and imu date.
    const Flash3dGnssImuFison::IMU_ptr last_imu_ptr = imu_buffer.back();
    state.time_stamp = last_imu_ptr->time_stamp;
    state.last_imu   = last_imu_ptr;


    state.g_= - acc_mean /acc_mean.norm() * 9.81;
    state.p_= gps_data->enu;
    state.R_= M3D::Identity();
    state.v_= V3D::Zero();
    state.gyro_bias = gyr_mean;
    
    V3D acc_sum_without_g_{0.0,0.0,0.0};
    for(const auto & elem :imu_buffer)
    {
        acc_sum_without_g_ += elem->acc + state.g_;
    }

    const V3D acc_mean_without_g_ = acc_sum_without_g_ / imu_buffer.size();
    V3D sum_err_acc{0.0,0.0,0.0};
    for(const auto & elem:imu_buffer)
    {
        sum_err_acc += ((elem->acc+state.g_)-acc_mean_without_g_).cwiseAbs2();
    }
    const V3D err_acc_std = (sum_err / (imu_buffer.size()-1)).cwiseSqrt();
    
    state.acc_bias = acc_mean_without_g_;

    V3D sum_err_gyro{0.0,0.0,0.0};
    for(const auto & elem:imu_buffer)
    {
        sum_err_gyro += (elem->gyro- gyr_mean).cwiseAbs2();
    }
    const V3D err_gyro_std = (sum_err / (imu_buffer.size()-1)).cwiseSqrt();

    this->gyro_std = err_gyro_std;
    this->acc_std  = err_acc_std;

    // Set covariance.
    state.cov_.setIdentity();
    state.cov_.block<3, 3>(0, 0) = 100. * Eigen::Matrix3d::Identity(); // position std: 10 m
    state.cov_.block<3, 3>(3, 3) = 100. * Eigen::Matrix3d::Identity(); // velocity std: 10 m/s
    // roll pitch std 10 degree.
    state.cov_.block<2, 2>(6, 6) = 10. * kDegreeToRadian * 10. * kDegreeToRadian * Eigen::Matrix2d::Identity();
    state.cov_(8, 8)             = 100. * kDegreeToRadian * 100. * kDegreeToRadian; // yaw std: 100 degree.
    // Acc bias.
    state.cov_.block<3, 3>(9, 9) =  1e-6 * Eigen::Matrix3d::Identity();
    // Gyro bias.
    state.cov_.block<3, 3>(12, 12) = 1e-4 * Eigen::Matrix3d::Identity();


    this->imu_inital_flag = true;
    return true;
}


bool Flash3dGnssImuFison::Initalizer::is_initalized()
{
    return this->imu_inital_flag;
}
