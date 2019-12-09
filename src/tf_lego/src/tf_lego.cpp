    #include <ros/ros.h>
    #include <tf/transform_broadcaster.h>
    #include <nav_msgs/Odometry.h>
        
    
    void poseCallback(nav_msgs::Odometry msg){
     static tf::TransformBroadcaster lego_br;
     tf::Transform transform;
     
     transform.setOrigin( tf::Vector3(msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z) );
     tf::Quaternion q(msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w);
     transform.setRotation(q);
     lego_br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/world", "odom"));
   }
     void ekfCallback(nav_msgs::Odometry msg){
     static tf::TransformBroadcaster lego_br_ekf;
     tf::Transform transform;
     transform.setOrigin( tf::Vector3(msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z) );
     tf::Quaternion q(msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w);
     transform.setRotation(q);
     lego_br_ekf.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/world", "odom_ekf"));
   }

   int main(int argc, char** argv){
     ros::init(argc, argv, "tf_lego");
   
     ros::NodeHandle node;
     ros::Subscriber sub_odom = node.subscribe("/ev3dev/diffDrv/odom", 10, &poseCallback);
     ros::Subscriber sub_ekf = node.subscribe("/EKF_Topic", 10, &ekfCallback);
     ros::spin();
     return 0;
   };
