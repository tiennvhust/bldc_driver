#include <ros/ros.h>
#include "bldc_driver/BLDC.hpp"
#include <memory>
#include <map>
#include <string>
#include <vector>
#include <std_msgs/Int32.h>
#include <std_msgs/Float64.h>
#include <std_srvs/Trigger.h>
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <diagnostic_msgs/KeyValue.h>

#define GEAR 50
#define RADIUS 0.0625

class BLDCWrapper 
{
private:
    std::unique_ptr<BLDC> motor_;
    ros::Subscriber speed_command_subscriber_;
    ros::ServiceServer init_motor_server_;
    ros::ServiceServer stop_motor_server_;
public:
    BLDCWrapper(ros::NodeHandle *nh)
    {
        motor_.reset(new BLDC(2));
        init_motor_server_ = nh->advertiseService(
            "init_motor", &BLDCWrapper::callbackInit, this);
        speed_command_subscriber_ = nh->subscribe<std_msgs::Float64> (
            "/robot_kist/left_joint_velocity_controller/command", 10, &BLDCWrapper::callbackSpeedCommand, this);
        stop_motor_server_ = nh->advertiseService(
            "stop_motor", &BLDCWrapper::callbackStop, this);
    }

    void callbackSpeedCommand(const std_msgs::Float64 &msg)
    {

        motor_->VelCmd(msg.data);
    }

    bool callbackInit(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
    {
        motor_init();
        res.success = true;
        res.message = "Successfully init motor.";
        return true;
    }

    bool callbackStop(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
    {
        stop();
        res.success = true;
        res.message = "Successfully stopped motor.";
        return true;
    }
    
    /*Init*/
    void motor_init()
    {
        motor_->MotorInit();
    }

    /*Stop*/
    void stop()
    {
        motor_->MotorStop();
    }
    
};
int main(int argc, char **argv)
{
    ros::init(argc, argv, "BLDC_driver");
    ros::NodeHandle nh;

    ros::AsyncSpinner spinner(4);
    spinner.start();

    BLDCWrapper BLDC_warpper(&nh);
    BLDC_warpper.motor_init();
    ROS_INFO("BLDC driver is now started");

    ros::waitForShutdown();
    BLDC_warpper.stop();
}