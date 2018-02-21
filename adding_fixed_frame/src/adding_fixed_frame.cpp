
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>

// this node executed per one robot 
// this node takes the robot name as argument 

std::string robot_name;

void odomCallback(const nav_msgs::OdometryConstPtr &odom_msg){
    
    //very inportant : static !!
    //initialization take time... 
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(odom_msg->pose.pose.position.x,
    odom_msg->pose.pose.position.y,odom_msg->pose.pose.position.z));
    
    tf::Quaternion q;
    q.setValue(odom_msg->pose.pose.orientation.x,
    odom_msg->pose.pose.orientation.y,odom_msg->pose.pose.orientation.z ,odom_msg->pose.pose.orientation.w);
    transform.setRotation(q);
    ROS_INFO("sending transform.\n");
    br.sendTransform(tf::StampedTransform(transform,ros::Time::now(),
    "map",robot_name+"/odom"));
};
int main(int argc, char **argv){
    ros::init(argc,argv,"mybot_tf_broadcaster");
    // robot name form : eg: robot1  
    robot_name=argv[1];
    ROS_INFO("strat node\n");
    ros::NodeHandle node;
    ros::Subscriber sub=node.subscribe(robot_name+"/odom",10,&odomCallback);
    ros::spin();
   
    return 0;

};




