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

/*
 *This part should be changed according to the setup of the real robot.
 *One must check motors' ID before define.
 */

#define LEFT_MOTOR 2
#define RIGHT_MOTOR 1
#define REAR_MOTOR 4

#include <cmath>

/*Convert m/s to rpm*/
int ms2rpm(double speed) {
    int rpm = speed / (2 * M_PI * RADIUS) * MOTOR_GEAR * 60;
    if (rpm < 0) return 0;
    if (rpm > RPM_MAX) return RPM_MAX;
    return rpm;
}

/*Convert rpm to m/s*/
double rpm2ms(int16_t rpm) {
    return (double)(rpm / MOTOR_GEAR) / 60 * (2 * M_PI * RADIUS);
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
    ros::ServiceServer set_dir_server_;

    ros::Timer motor_status_timer_;

    double publish_motor_status_frequency_;
    bool motor_dir_;
public:
    BLDCWrapper(ros::NodeHandle *nh)
    {
        #ifdef LEFT_MOTOR
        left.reset(new BLDC(LEFT_MOTOR));
        left_speed_command_subscriber_ = nh->subscribe<std_msgs::Float64> (
            "/robot_kist/left_joint_velocity_controller/command", 10, &BLDCWrapper::callbackSpeedCommandLeft, this);
        #endif

        #ifdef RIGHT_MOTOR
        right.reset(new BLDC(RIGHT_MOTOR));
        right_speed_command_subscriber_ = nh->subscribe<std_msgs::Float64> (
            "/robot_kist/right_joint_velocity_controller/command", 10, &BLDCWrapper::callbackSpeedCommandRight, this);
        #endif

        #ifdef REAR_MOTOR
        rear.reset(new BLDC(REAR_MOTOR));
        rear_speed_command_subscriber_ = nh->subscribe<std_msgs::Float64> (
            "/robot_kist/back_joint_velocity_controller/command", 10, &BLDCWrapper::callbackSpeedCommandRear, this);
        #endif

        init_motor_server_ = nh->advertiseService(
            "/bldc_driver/init_motor", &BLDCWrapper::callbackInit, this);

        stop_motor_server_ = nh->advertiseService(
            "/bldc_driver/stop_motor", &BLDCWrapper::callbackStop, this);

        if (!ros::param::get("~motor_dir_", motor_dir_)) {motor_dir_ = 1;}

        set_dir_server_ = nh->advertiseService(
            "/bldc_driver/set_moving_direction", &BLDCWrapper::callbackSetDir, this);

        if (!ros::param::get("~publish_motor_status_frequency", publish_motor_status_frequency_)) {publish_motor_status_frequency_ = 1.0;}

        motor_status_publisher_ = nh->advertise<sensor_msgs::JointState> (
            "/robot_kist/joint_states", 10);

        motor_status_timer_ = nh->createTimer(
            ros::Duration(1.0 / publish_motor_status_frequency_),
           &BLDCWrapper::publishMotorStatus, this);
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
        sensor_msgs::JointState msg;
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
    void motorSpeedUpdate(sensor_msgs::JointState &data)
    {
        /*Get main data from motors*/
        data.name.push_back("Motor Speed");
        #ifdef LEFT_MOTOR
        int16_t left_rpm;
        left->ReqRpmData(left_rpm);
        data.velocity.push_back(rpm2ms(left_rpm));
        #endif

        #ifdef RIGHT_MOTOR
        int16_t right_rpm;
        right->ReqRpmData(right_rpm);
        data.velocity.push_back(rpm2ms(right_rpm));
        #endif

        #ifdef REAR_MOTOR
        int16_t rear_rpm;
        rear->ReqRpmData(rear_rpm);
        data.velocity.push_back(rpm2ms(rear_rpm));
        #endif
    }

    /*Start motors*/
    void motorInit()
    {
        #ifdef LEFT_MOTOR
        left->MotorInit();
        #endif

        #ifdef RIGHT_MOTOR
        right->MotorInit();
        #endif

        #ifdef REAR_MOTOR
        rear->MotorInit();
        #endif
        stop();
    }

    bool callbackStop(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
    {
        stop();
        res.success = true;
        res.message = "Successfully stopped motor.";
        return true;
    }

    /*Stop*/
    void stop()
    {
        #ifdef LEFT_MOTOR
        left->MotorStop();
        #endif

        #ifdef RIGHT_MOTOR
        right->MotorStop();
        #endif

        #ifdef REAR_MOTOR
        rear->MotorStop();
        #endif
    }

    bool callbackSetDir(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
    {
        #ifdef LEFT_MOTOR
        left->SetSignCmd(motor_dir_);
        #endif

        #ifdef RIGHT_MOTOR
        right->SetSignCmd(motor_dir_);
        #endif

        #ifdef REAR_MOTOR
        rear->SetSignCmd(motor_dir_);
        #endif

        res.success = true;
        res.message = "Successfully changed moving direction.";
        return true;
    }
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
    BLDC_warpper.stop();
}