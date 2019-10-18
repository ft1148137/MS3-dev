#ifndef LINE_FLOWER_H_
#define LINE_FLOWER_H_
#include "ros/ros.h"
#include <realtime_tools/realtime_publisher.h>
#include <std_msgs/ColorRGBA.h>
#include <std_msgs/Header.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/JointState.h>
#include "line_follower/pid.h"
class line_follower
{	public:
	line_follower(ros::NodeHandle &nh);
	~line_follower();
	void init();
	struct geometry_odom_vel{
		std_msgs::Header header;
		geometry_msgs::Twist postion;
		geometry_msgs::Twist velocity;
		};
	private:
	ros::NodeHandle n;
	ros::Subscriber color_sensor_sub;
	ros::Subscriber geometry_odom_vel_sub;
	ros::Publisher motor_control_publisher;
	ros::Timer data_fusion_callback_timer;
	ros::Timer angle_motion_control_timer;
	PID* pid_angle;
	
	std_msgs::ColorRGBA color_now;
	std::vector<geometry_msgs::Twist> motor_encoder_buffer;
	geometry_msgs::Twist control_message_to_pub;
	
	double PID_PARM[3];
	double PID_state[3];
	double COLOR_RGB_THRESHOLD[3];
	double COLOR_RGB_CALIB[3];
	int turned = 0;
	int stoped = 0;

	void geometry_odom_vel_callback(geometry_msgs::Twist geo_msg);
	void color_sensor_callback(std_msgs::ColorRGBA color_);	
	void data_fusion_callback();
	void motor_controller_callback(const ros::TimerEvent& event);
	double PID_regler(const double &error_);
	
	};
#endif
