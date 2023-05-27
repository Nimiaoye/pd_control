/**
 * @file offb_node.cpp
 * @brief Offboard control example node, written with MAVROS version 0.19.x, PX4 Pro Flight
 * Stack and tested in Gazebo Classic SITL
 */
//速度主题用/mavros/local_position/velocity_local
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <geometry_msgs/TwistStamped.h>   // velocity subsribe
//#include "stdafx.h"
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <math.h>
#define PI       3.14159265358979323846 
typedef struct
{
	float w, x, y, z;
}Quat_t;//四元素结构体
typedef struct
{
	float Pitch;
	float Roll;
	float Yaw;
}Euler_t;//欧拉角结构体
//欧拉角转四元素
//euler_angle：输入欧拉角
//q1：输出四元素
int Conversion_Euler_to_Quaternion(Quat_t q1, Euler_t euler_angle)
{
	euler_angle.Yaw = euler_angle.Yaw *  PI / 180;
	euler_angle.Pitch = euler_angle.Pitch * PI / 180;
	euler_angle.Roll = euler_angle.Roll * PI / 180;
	double c1 = acos(euler_angle.Yaw / 2);
	double s1 = asin(euler_angle.Yaw / 2);
	double c2 = acos(euler_angle.Pitch / 2);
	double s2 = asin(euler_angle.Pitch / 2);
	double c3 = acos(euler_angle.Roll / 2);
	double s3 = asin(euler_angle.Roll / 2);
	double c1c2 = c1 * c2;
	double s1s2 = s1 * s2;
	q1.w = (c1c2 * c3 + s1s2 * s3);
	q1.x = (c1c2 * s3 + s1s2 * c3);
	q1.y = (s1 * c2 * c3 + c1 * s2 * s3);
	q1.z = (c1 * s2 * c3 - s1 * c2 * s3);
	return 0;
}


mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

geometry_msgs::PoseStamped local_pos;
void local_pos_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    local_pos = *msg;
}

geometry_msgs::TwistStamped local_vel;
void local_vel_cb(const geometry_msgs::TwistStamped::ConstPtr& msg){
    local_vel = *msg;
}

int main(int argc, char **argv)
{
    Euler_t euler = { 0,0,0 };
    Quat_t quat1 = { 1,0,0,0 };
    float plustheta=0;
    int flag_step0=0;
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::Subscriber local_pos_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("mavros/local_position/pose", 10, local_pos_cb);
            // 订阅无人机当前位置（反馈消息） 
    ros::Publisher local_attitude_pub = nh.advertise<mavros_msgs::AttitudeTarget>
            ("mavros/setpoint_raw/attitude", 7);  //我自己改的，发布推理与姿态角
    ros::Subscriber local_vel_sub = nh.subscribe<geometry_msgs::TwistStamped>
            ("mavros/local_position/velocity_local", 10, local_vel_cb);//无人机三轴速度
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(100.0);

    // wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    /*geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 2;*/
    mavros_msgs::AttitudeTarget attitude;
    attitude.orientation.x = 0;
    attitude.orientation.y = 0;
    attitude.orientation.z = 0;
    attitude.orientation.w = 1;
    attitude.thrust = 0.725;//推力占比

    //geometry_msgs::TwistStamped velocity;
    ROS_INFO("local_vel.twist.linear.z == %f",local_vel.twist.linear.z);
    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
        local_attitude_pub.publish(attitude);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;
    
    ros::Time last_request = ros::Time::now();
    
    int step = 0;
    int sametimes = 0; //（用于记录累计时间）
    float Kpz = 1;
    float Kpvz = 1; //z轴稳定控制参数
    float Vdes_z =0;
    while(ros::ok()){
        if( current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0))){
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent){
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        } else {
            if( !current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0))){
                if( arming_client.call(arm_cmd) &&
                    arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
            else    //  无人机 Offboard enabled && Vehicle armed 后
            {       
                    
                switch (step)
                {
                case 0: 
                    //take off to 1m  第一步飞到1米
                    attitude.orientation.x = 0;
                    attitude.orientation.y = 0;
                    attitude.orientation.z = 0;
                    attitude.orientation.w = 1;

                    Vdes_z = Kpz*(1-local_pos.pose.position.z);
                    attitude.thrust = 0.7+ Kpvz*(Vdes_z-local_vel.twist.linear.z);
                    /*if(attitude.thrust > 0.75)
                        attitude.thrust = 0.75;
                    if(attitude.thrust < 0.5)
                        attitude.thrust = 0.5;*/
        
                    ROS_INFO("thrust1:%f",attitude.thrust);
                    ROS_INFO("aixis_z:%f",local_pos.pose.position.z);
                    ROS_INFO("local_vel.twist.linear.z == %f",local_vel.twist.linear.z);
                    if (local_pos.pose.position.z > 0.9 && local_pos.pose.position.z < 1.2)
                    {   

                        if (sametimes < 2000)
                        sametimes++;
                        else if (sametimes > 2000 && sametimes < 7000)
                        {
                            sametimes = 0;
                            step = 1;
                        attitude.orientation.x = 0;
                        attitude.orientation.y = 0;
                        attitude.orientation.z = 0;
                        attitude.orientation.w = 1;
                        ROS_INFO("yes you are good!ffffffffffffffffate!");
                        }
                        else
                            sametimes++;
                    }
                    else
                    {
                        sametimes = 0;
                    }
                    local_attitude_pub.publish(attitude);

                    break;
                case 1:   //在空中一米处飞到（2,1,0）的位置
                    while(1){
                        plustheta=10*(2.0-local_pos.pose.position.x);
                        euler.Roll=plustheta;
                        Conversion_Euler_to_Quaternion(quat1,euler);
                        attitude.orientation.x = quat1.x;
                        attitude.orientation.y = quat1.y;
                        attitude.orientation.z = quat1.z;
                        attitude.orientation.w = quat1.w;
                        ROS_INFO("step5 enabled");
                        local_attitude_pub.publish(attitude);
                        if (local_pos.pose.position.x > 1.9 && local_pos.pose.position.x < 2.1)
                                        {
                        if (sametimes > 20)
                        {
                            step = 5;
                            break;
                        }
                        else
                            sametimes++;
                                        }
                    else
                    {
                        sametimes = 0;
                    }
                            }   


                    
                    break;
                
                case 5:   // 准备降落
                    offb_set_mode.request.custom_mode = "AUTO.LAND";
                    if (current_state.mode != "AUTO.LAND" && (ros::Time::now() - last_request > ros::Duration(5.0)))
                    {

                        if (set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent)
                        {
                            ROS_INFO("AUTO.LAND enabled");
                        }
                        last_request = ros::Time::now();
                    }
                    break;
                default:
                    break;
                }//switch loop
            }
        }
        

        local_attitude_pub.publish(attitude);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}


