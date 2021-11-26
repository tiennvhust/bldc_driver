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
#include <sensor_msgs/JointState.h>

#define _USE_MATH_DEFINES

// #define LEFT_MOTOR
#define RIGHT_MOTOR
// #define REAR_MOTOR
 
#include <cmath>

/*Convert m/s to rpm*/
int ms2rpm(double speed) {
    return speed / (2 * M_PI * RADIUS) * MOTOR_GEAR * 60;
}

/*Convert rpm to m/s*/
double rpm2ms(int rpm) {
    return (rpm / MOTOR_GEAR) / 60 * (2 * M_PI * RADIUS);
}

class BLDCWrapper 
{
private:
    std::unique_ptr<BLDC> left;
    std::unique_ptr<BLDC> right;
    std::unique_ptr<BLDC> rear;
    ros::Subscriber left_speed_command_subscriber_;
    ros::Subscriber right_speed_command_subscriber_;
    ros::Subscriber rear_speed_command_subscriber_;
    ros::Publisher motor_status_publisher_;
    ros::ServiceServer init_motor_server_;
    ros::ServiceServer stop_motor_server_;

    double publish_motor_status_frequency_;
public:
    BLDCWrapper(ros::NodeHandle *nh)
    {
#ifdef LEFT_MOTOR
        left.reset(new BLDC(1));
        left_speed_command_subscriber_ = nh->subscribe<std_msgs::Float64> (
            "/robot_kist/left_joint_velocity_controller/command", 10, &BLDCWrapper::callbackSpeedCommandLeft, this);
#endif
#ifdef RIGHT_MOTOR
        right.reset(new BLDC(2));
        right_speed_command_subscriber_ = nh->subscribe<std_msgs::Float64> (
            "/robot_kist/right_joint_velocity_controller/command", 10, &BLDCWrapper::callbackSpeedCommandRight, this);
#endif
#ifdef REAR_MOTOR
        rear.reset(new BLDC(3));
        rear_speed_command_subscriber_ = nh->subscribe<std_msgs::Float64> (
            "/robot_kist/back_joint_velocity_controller/command", 10, &BLDCWrapper::callbackSpeedCommandRear, this);
#endif  
        if (!ros::param::get("~publish_motor_status_frequency", publish_motor_status_frequency_))
        {
            publish_motor_status_frequency_ = 1.0;
        }

        init_motor_server_ = nh->advertiseService(
            "init_motor", &BLDCWrapper::callbackInit, this);

        motor_status_publisher_ = nh->advertise<std_msgs::JointState> (
            "/robot_kist/joint_states", 10);

        motor_status_timer_ = nh->createTimer(
            ros::Duration(1.0 / publish_motor_status_frequency_),
            &BLDCWrapper::publishMotorStatus, this);


        // stop_motor_server_ = nh->advertiseService(
        //     "stop_motor", &BLDCWrapper::callbackStop, this);
    }

#ifdef LEFT_MOTOR
    /*Set speed for left motor*/
    void callbackSpeedCommandLeft(const std_msgs::Float64::ConstPtr &msg)
    {
        left->VelCmd(ms2rpm(msg->data));
    }
#endif
#ifdef RIGHT_MOTOR
    /*Set speed for right motor*/
    void callbackSpeedCommandRight(const std_msgs::Float64::ConstPtr &msg)
    {
        right->VelCmd(ms2rpm(msg->data));
    }
#endif
#ifdef REAR_MOTOR
    /*Set speed for rear motor*/
    void callbackSpeedCommandRear(const std_msgs::Float64::ConstPtr &msg)
    {
        rear->VelCmd(ms2rpm(msg->data));
    }
#endif

    /*Publish motor status*/
    void publishMotorStatus(const ros::TimerEvent &event)
    {
        std_msgs::JointState msg;
        motorSpeedUpdate(msg);
        motor_status_publisher_.publish(msg);
    }

    bool callbackInit(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
    {
        motorInit();
        res.success = true;
        res.message = "Successfully start motors.";
        return true;
    }

    /*Update motor speed*/
    void motorSpeedUpdate(std_msgs::JointState &data)
    {
        /*Get main data from motors*/
#ifdef LEFT_MOTOR
        left->ReqMainData();
        data.velocity[0] = rpm2ms(left->ReadRpm());
#endif
#ifdef RIGHT_MOTOR
        right->ReqMainData();
        data.velocity[1] = rpm2ms(right->ReadRpm());
#endif
#ifdef REAR_MOTOR
        rear->ReqMainData();
        data.velocity[2] = rpm2ms(rear->ReadRpm());
#endif
    }

    /*Start motors*/
    void motorInit()
    {
#ifdef LEFT_MOTOR
        left->MotorInit();
        left->MotorStop();
#endif
#ifdef RIGHT_MOTOR
        right->MotorInit();
        right->MotorStop();
#endif
#ifdef REAR_MOTOR
        rear->MotorInit();
        rear->MotorStop();
#endif
    }

    // bool callbackStop(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
    // {
    //     stop();
    //     res.success = true;
    //     res.message = "Successfully stopped motor.";
    //     return true;
    // }

    // /*Stop*/
    // void stop()
    // {
    //     left->MotorStop();
    // }
    
};
int main(int argc, char **argv)
{
    ros::init(argc, argv, "BLDC_driver");
    ros::NodeHandle nh;

    ros::AsyncSpinner spinner(4);
    spinner.start();

    BLDCWrapper BLDC_warpper(&nh);
    BLDC_warpper.motorInit();
    ROS_INFO("BLDC driver is now started");

    ros::waitForShutdown();
    // BLDC_warpper.stop();
}