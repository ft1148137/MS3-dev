#include "line_follower/line_follower.h"
line_follower::line_follower(ros::NodeHandle &nh):n(nh){
	
	color_sensor_sub = n.subscribe<std_msgs::ColorRGBA>("/color",10,&line_follower::color_sensor_callback,this);
	motor_control_publisher = n.advertise<geometry_msgs::Twist>("/ev3dev/diffDrv/cmd_vel",10);
	geometry_odom_vel_sub = n.subscribe<geometry_msgs::Twist>("sensor_msgs/JointState",10,&line_follower::geometry_odom_vel_callback,this);
	angle_motion_control_timer = n.createTimer(ros::Duration(0.1),&line_follower::motor_controller_callback,this);
	init();
	pid_angle = new PID(0.2,1,-1,PID_PARM[0],PID_PARM[1], PID_PARM[2]);
	}
void line_follower::init(){
	PID_PARM[0] = 0.4;
	PID_PARM[1] = 0.15;
	PID_PARM[2] = 0;
	
	//COLOR_RGB_CALIB[0] = 200;
	//COLOR_RGB_CALIB[1] = 200;
	COLOR_RGB_CALIB[2] = 100;
	
	COLOR_RGB_THRESHOLD[0] = 25;
	COLOR_RGB_THRESHOLD[1] = 40;
	COLOR_RGB_THRESHOLD[2] = 30;
	control_message_to_pub.linear.x = 0.04;
	}
void line_follower::color_sensor_callback(std_msgs::ColorRGBA color_){
	color_now = color_;
	}
void line_follower::geometry_odom_vel_callback(geometry_msgs::Twist geo_msg){}
void line_follower::motor_controller_callback(const ros::TimerEvent& event){
	float error_z ;
	ROS_INFO("color:");
	ROS_INFO("R: %f",color_now.r);
	ROS_INFO("G: %f",color_now.g);
	ROS_INFO("B: %f",color_now.b);

	if(color_now.r + color_now.g + color_now.b  < 90){
		if(stoped < 5){
	    stoped ++;
		control_message_to_pub.linear.x = 0;
		control_message_to_pub.angular.z = 0;
			}
		else if(turned < 15){
		turned ++;
		control_message_to_pub.linear.x = 0;
		control_message_to_pub.angular.z = -1;
		ROS_INFO("turn");
			}
		else{
		control_message_to_pub.linear.x = 0.03;
		control_message_to_pub.angular.z = 0;
		ROS_INFO("turn END");
		}
	}
	else{
		turned = false;
		stoped = 0;
		error_z = -(color_now.b-((COLOR_RGB_CALIB[2] - COLOR_RGB_THRESHOLD[2])/2))/(COLOR_RGB_CALIB[2] -COLOR_RGB_THRESHOLD[2]);
		ROS_INFO("ERROR_z: %f",error_z);
		control_message_to_pub.angular.z = PID_regler(error_z);
				control_message_to_pub.linear.x = 0.03;

			}	

		motor_control_publisher.publish(control_message_to_pub);
	};
double line_follower::PID_regler(const double &error_){
	
	return (pid_angle->calculate(0,error_));
	}
int main(int argc, char **argv)
{	
	ros::init(argc, argv, "Line_follower");
	ros::NodeHandle nh;
	line_follower* line_follower_ = new line_follower(nh);
	ros::Rate r(20);
	while(ros::ok()){
		ros::spinOnce();
		r.sleep();
		
		}
	return 0;
	}
