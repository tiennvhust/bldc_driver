#include <ros/ros.h>
#include "bldc_driver/vk_motors.hpp"
#include <memory>
#include <map>
#include <string>
#include <vector>
#include <std_msgs/Int32.h>
#include <std_msgs/Float64.h>
#include <std_srvs/Trigger.h>
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <diagnostic_msgs/KeyValue.h>
#include <sensor_msgs/JointState.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "vk_motors");
    ros::NodeHandle nh;

    ros::AsyncSpinner spinner(4);
    spinner.start();

    BLDCWrapper BLDC_warpper(&nh);
    BLDC_warpper.motorInit();
    ROS_INFO("Robot motor(s) is now started.");

    ros::waitForShutdown();
    BLDC_warpper.stop();
}
