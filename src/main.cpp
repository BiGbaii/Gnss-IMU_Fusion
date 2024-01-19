#include<rclcpp/rclcpp.hpp>
#include"flash3d_gnss_imu_fusion/gins.h"

int main(int argc , char ** argv)
{
    rclcpp::init(argc,argv);
    printf("*********GNIS**********\n");
    auto node = std::make_shared<GinsNode>("GNIS");

    rclcpp::spin(node);
    rclcpp::shutdown();


    return 0;
}