#include "bldc_driver/vk_motors.hpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "vk_motors");

    ros::NodeHandle nh;

    ros::AsyncSpinner spinner(4);

    spinner.start();

    BLDCWrapper BLDC_warpper(&nh);

    ROS_INFO("vk_motors node.");

    ros::waitForShutdown();
}
