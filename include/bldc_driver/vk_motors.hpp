#ifndef VK_MOTORS_H
#define VK_MOTORS_H

#include <ros/ros.h>
#include "bldc_driver/BLDC.hpp"

#include <memory>
#include <map>
#include <string>
#include <vector>

#include <std_msgs/Int32.h>
#include <std_msgs/Float64.h>
#include <std_srvs/Trigger.h>
#include <std_srvs/SetBool.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Float64MultiArray.h>

#include <diagnostic_msgs/DiagnosticStatus.h>
#include <diagnostic_msgs/KeyValue.h>

#define _USE_MATH_DEFINES

/*
 *This part should be changed according to the setup of the real robot.
 *One must check motors' ID before define.
 */

#define LEFT_MOTOR 2
#define RIGHT_MOTOR 1
#define REAR_MOTOR 4
#define DEFAULT_FREQUENCY 50
#define NOM_CODE 12

#include <cmath>

/*Convert m/s to rpm*/
int16_t ms2rpm(double speed) {
    int16_t rpm = speed / (2 * M_PI * RADIUS) * MOTOR_GEAR * 60;
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
    std::unique_ptr<BLDC> left; /*left motor*/

    std::unique_ptr<BLDC> right; /*right motor*/

    std::unique_ptr<BLDC> rear; /*rear motor*/
    
    ros::Subscriber left_speed_command_subscriber_;

    ros::Subscriber right_speed_command_subscriber_;

    ros::Subscriber rear_speed_command_subscriber_;

    ros::Subscriber robot_status_subscriber_;
    
    ros::Publisher motor_status_publisher_;

    ros::ServiceServer set_dir_server_;

    ros::Timer publish_timer_;

    double publish_frequency_;

    bool readyStatus; /*ready status*/

public:
    BLDCWrapper(ros::NodeHandle *nh) : 
            readyStatus(false)
    {
        #ifdef LEFT_MOTOR
        left.reset(new BLDC(LEFT_MOTOR));
        left_speed_command_subscriber_ = nh->subscribe<std_msgs::Float64> (
            "/robot_kist/joint_3_velocity/command", 10, &BLDCWrapper::callbackSpeedCommandLeft, this);
        // left_speed_command_subscriber_ = nh->subscribe<std_msgs::Float64> (
            // "/robot_kist/left_joint_velocity_controller/command", 10, &BLDCWrapper::callbackSpeedCommandLeft, this);
        #endif

        #ifdef RIGHT_MOTOR
        right.reset(new BLDC(RIGHT_MOTOR));
        right_speed_command_subscriber_ = nh->subscribe<std_msgs::Float64> (
            "/robot_kist/joint_2_velocity/command", 10, &BLDCWrapper::callbackSpeedCommandRight, this);
        // right_speed_command_subscriber_ = nh->subscribe<std_msgs::Float64> (
            // "/robot_kist/right_joint_velocity_controller/command", 10, &BLDCWrapper::callbackSpeedCommandRight, this);
        #endif

        #ifdef REAR_MOTOR
        rear.reset(new BLDC(REAR_MOTOR));
        rear_speed_command_subscriber_ = nh->subscribe<std_msgs::Float64> (
            "/robot_kist/joint_1_velocity/command", 10, &BLDCWrapper::callbackSpeedCommandRear, this);
        // rear_speed_command_subscriber_ = nh->subscribe<std_msgs::Float64> (
            // "/robot_kist/back_joint_velocity_controller/command", 10, &BLDCWrapper::callbackSpeedCommandRear, this);
        #endif

        robot_status_subscriber_ = nh->subscribe<std_msgs::UInt8> (
            "robot_status", 10, &BLDCWrapper::callbackRobotStatus, this);

        set_dir_server_ = nh->advertiseService(
            "/vk_motors/set_moving_direction", &BLDCWrapper::callbackSetDir, this);

        motor_status_publisher_ = nh->advertise<std_msgs::Float64MultiArray> (
            "wheels_speed", 10);

        if (!ros::param::get("~publish_motor_status_frequency", publish_frequency_))
            publish_frequency_ = DEFAULT_FREQUENCY;

        publish_timer_ = nh->createTimer(
            ros::Duration(1.0 / publish_frequency_),
           &BLDCWrapper::publishMotorSpeed, this);
    }

    #ifdef LEFT_MOTOR
    /*Set speed for left motor*/
    void callbackSpeedCommandLeft(const std_msgs::Float64::ConstPtr &msg)
    {
        if (readyStatus) left->VelCmd(ms2rpm(msg->data));
        else ROS_INFO("Motor(s) is NOT Initialized\n");
    }
    #endif

    #ifdef RIGHT_MOTOR
    /*Set speed for right motor*/
    void callbackSpeedCommandRight(const std_msgs::Float64::ConstPtr &msg)
    {
        if (readyStatus) right->VelCmd(ms2rpm(msg->data));
        else ROS_INFO("Motor(s) is NOT Initialized\n");
    }
    #endif

    #ifdef REAR_MOTOR
    /*Set speed for rear motor*/
    void callbackSpeedCommandRear(const std_msgs::Float64::ConstPtr &msg)
    {
        if (readyStatus) rear->VelCmd(ms2rpm(msg->data));
        else ROS_INFO("Motor(s) is NOT Initialized\n");
    }
    #endif

    /*Publish motor status*/
    void publishMotorSpeed(const ros::TimerEvent &event)
    {
        std_msgs::Float64MultiArray msg;
        getMotorSpeed(msg);
        motor_status_publisher_.publish(msg);
    }

    /*Robot status update*/
    void callbackRobotStatus(const std_msgs::UInt8::ConstPtr &msg)
    {
        if (msg->data == NOM_CODE) motorInit();
        else readyStatus = false;
    }

    /*Update motors speed*/
    void getMotorSpeed(std_msgs::Float64MultiArray &msg)
    {
        if (readyStatus)
        {
            #ifdef LEFT_MOTOR
            int16_t left_rpm;
            left->ReqRpmData(left_rpm);
            msg.data.push_back(rpm2ms(left_rpm));
            #endif
        
            #ifdef RIGHT_MOTOR
            int16_t right_rpm;
            right->ReqRpmData(right_rpm);
            msg.data.push_back(rpm2ms(right_rpm));
            #endif
        
            #ifdef REAR_MOTOR
            int16_t rear_rpm;
            rear->ReqRpmData(rear_rpm);
            msg.data.push_back(rpm2ms(rear_rpm));
            #endif
        }
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

        readyStatus = true;

        ROS_INFO("Motor(s) is initialized.\n");
    }

    bool callbackSetDir(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res)
    {
        if (readyStatus)
        {
            #ifdef LEFT_MOTOR
            left->SetSignCmd(req.data);
            #endif
        
            #ifdef RIGHT_MOTOR
            right->SetSignCmd(req.data);
            #endif
        
            #ifdef REAR_MOTOR
            rear->SetSignCmd(req.data);
            #endif
        
            res.message = req.data ? "Successfully changed moving direction to CCW." : "Successfully changed moving direction to CW.";
        
            res.success = true;
        }
        else res.message = "Changing moving direction FAILED! Robot is NOT in READY state.";

        return true;
    }
};

#endif