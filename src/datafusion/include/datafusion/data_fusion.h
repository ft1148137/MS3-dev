#ifndef DATA_FUSION_H
#define DATA_FUSION_H

#define Nsta 7  
#define Mobs 7  

#include "iostream"
#include <datafusion/TinyEKF.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>




class data_fusion : public TinyEKF{
	public:

	data_fusion(ros::NodeHandle &nh);
	void data_fusion_core();

	bool reset = false;

	private:
	ros::NodeHandle n;
	ros::Publisher EKF_Pub;
	
	typedef message_filters::sync_policies::ApproximateTime<nav_msgs::Odometry,sensor_msgs::Imu> syncPolic_;
	message_filters::Subscriber<nav_msgs::Odometry> encoder_sub;
	message_filters::Subscriber<sensor_msgs::Imu> IMU_sub;
	typedef message_filters::Synchronizer<syncPolic_> Sync;
	boost::shared_ptr<Sync> sync;
	tf::TransformBroadcaster odom_ekf;
	
	tf::TransformListener listener;
	void model(double fx[Nsta], double F[Nsta][Nsta], double hx[Mobs], double H[Mobs][Nsta]);
	void encoder_callback(nav_msgs::Odometry data);
	void gyro_callback(sensor_msgs::Imu data);
	void sync_callback(const nav_msgs::Odometry::ConstPtr&  data_odom,const sensor_msgs::Imu::ConstPtr& data_imu);
	
	std::vector<nav_msgs::Odometry> encoder_data_buffer;
	std::vector<sensor_msgs::Imu> gyro_data_buffer;
    geometry_msgs::Vector3Stamped gyro_translation_link;
	geometry_msgs::QuaternionStamped gyro_rotation_link;
	
	geometry_msgs::Vector3Stamped gyro_translation_base;
	geometry_msgs::QuaternionStamped gyro_rotation_base;
	double Freq_ ;

	};
#endif //DATA_FUSION_


