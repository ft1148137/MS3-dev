#include "datafusion/data_fusion.h"

data_fusion::data_fusion(ros::NodeHandle &nh):n(nh)
{
	encoder_sub.subscribe(n,"/ev3dev/diffDrv/odom",1);
	EKF_Pub = n.advertise<geometry_msgs::Twist>("/EKF_Topic",10);
	IMU_sub.subscribe(n,"/imu/data",1);
	sync.reset(new Sync(syncPolic_(10),encoder_sub,IMU_sub));
    sync -> registerCallback(boost::bind(&data_fusion::sync_callback,this,_1,_2));	
	
	this->setQ(0, 0, 1);
    this->setQ(1, 1, 1);
    this->setQ(2, 2, 1);
    this->setQ(3, 3, 1);
    this->setQ(4, 4, 1);
    this->setQ(5, 5, 1);
    this->setQ(6, 6, 1);
    
    this->setR(0, 0, 0.1);
    this->setR(1, 1, 0.1);
    this->setR(2, 2, 0.1);
    this->setR(3, 3, 0.1);
    this->setR(4, 4, 0.1);
    this->setR(5, 5, 0.1);
    this->setR(6, 6, 0.1);
	
	
	}
																


	
	
void data_fusion::model(double fx[Nsta], double F[Nsta][Nsta], double hx[Mobs], double H[Mobs][Nsta])
        {   
			Freq_ = 10;
			
            fx[0] = this->x[0];
            fx[1] = this->x[1];
            fx[2] = this->x[2];
            
            fx[3] = this->x[3];
            fx[4] = this->x[4];
            fx[5] = this->x[5];
            
            fx[6] = this->x[6];

            F[0][0] = 1;
            F[0][1] = 1/Freq_;
            F[0][2] = 0.5/(Freq_*Freq_);
            F[1][1] = 1;
            F[1][2] = 1/Freq_;
            F[2][2] = 1;
            
            F[3][3] = 1;
            F[3][4] = 1/Freq_;
            F[3][5] = 0.5/(Freq_*Freq_);
            F[4][4] = 1;
            F[4][5] = 1/Freq_;
            F[5][5] = 1;
            
            F[6][6] = 1;



            hx[0] = this->x[0]; 
            hx[1] = this->x[1]; 
            hx[2] = this->x[2]; 
            
            hx[3] = this->x[3]; 
            hx[4] = this->x[4]; 
            hx[5] = this->x[5];      
            hx[6] = this->x[6]; 

            H[0][0] = 1;   
            H[1][1] = 1;  
            H[2][2] = 1;  
            H[3][3] = 1;   
            H[4][4] = 1;  
            H[5][5] = 1; 
            H[6][6] = 1;   
    
        }	

void data_fusion::data_fusion_core(){
	   ROS_INFO("size; %d  %d",gyro_data_buffer.size(),encoder_data_buffer.size());
	   if(gyro_data_buffer.size() && encoder_data_buffer.size()){
			   gyro_translation_link.header = gyro_data_buffer[0].header;
			   gyro_translation_link.vector = gyro_data_buffer[0].linear_acceleration;
			   gyro_rotation_link.header = gyro_data_buffer[0].header;
			   gyro_rotation_link.quaternion = gyro_data_buffer[0].orientation;
			   try
			   {
			   listener.transformVector("base_link",gyro_translation_link,gyro_translation_base);
			   listener.transformQuaternion("base_link",gyro_rotation_link,gyro_rotation_base);}
			   catch(tf::TransformException& ex){
				   ROS_ERROR("%s",ex.what());
				   }
			   double sensor_data[7] = {
										encoder_data_buffer[0].pose.pose.position.x,
										encoder_data_buffer[0].twist.twist.linear.x,
										gyro_translation_base.vector.x,
										encoder_data_buffer[0].pose.pose.position.y,
										encoder_data_buffer[0].twist.twist.linear.y,
										gyro_translation_base.vector.y,
										gyro_rotation_base.quaternion.z
										};
			 	step(sensor_data);
				std::cout<<"X: "<<getX(0)<<"\n";
				std::cout<<"X': "<<getX(1)<<"\n";
				std::cout<<"X'': "<<getX(2)<<"\n";
				encoder_data_buffer.erase(encoder_data_buffer.begin());
				gyro_data_buffer.erase(gyro_data_buffer.begin());
			   }
			
	}	
void data_fusion::encoder_callback(nav_msgs::Odometry data){
	 encoder_data_buffer.push_back(data);
	}	
void data_fusion::gyro_callback(sensor_msgs::Imu data){
	 gyro_data_buffer.push_back(data);
		}
void data_fusion::sync_callback(const nav_msgs::Odometry::ConstPtr& data_odom,const sensor_msgs::Imu::ConstPtr&  data_imu){
	 encoder_data_buffer.push_back(*data_odom);
	 gyro_data_buffer.push_back(*data_imu);
	}
int main(int argc, char **argv)
{
	ros::init(argc, argv, "data_fusion");
	ros::NodeHandle nh;
	data_fusion* data_fusion_ = new data_fusion(nh);
	ros::Rate r(100);
	while(ros::ok()){
		data_fusion_ -> data_fusion_core();
		ros::spinOnce();
		r.sleep();	
		}
 return 0;
}

	
