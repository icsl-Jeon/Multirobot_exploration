#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_listener.h>


ros::Publisher pub;
tf::TransformListener *listener;
sensor_msgs::PointCloud2 output, output1, output2;
pcl::PointCloud<pcl::PointXYZ> output_pcl, output1_pcl, output2_pcl;

void cloud_cb1(const sensor_msgs::PointCloud2ConstPtr& input1) {
    listener->waitForTransform("/map", (*input1).header.frame_id, (*input1).header.stamp, ros::Duration(5.0));
    pcl_ros::transformPointCloud("/map", *input1, output1, *listener);
    pcl::fromROSMsg(output1, output1_pcl);
    output_pcl = output1_pcl;
    output_pcl += output2_pcl;
    pcl::toROSMsg(output_pcl, output);
    pub.publish(output);

}

void cloud_cb2(const sensor_msgs::PointCloud2ConstPtr& input2) {
    listener->waitForTransform("/map", (*input2).header.frame_id, (*input2).header.stamp, ros::Duration(5.0));
    pcl_ros::transformPointCloud("/map", *input2, output2, *listener);
    pcl::fromROSMsg(output2, output2_pcl);
    output_pcl = output2_pcl;
    output_pcl += output1_pcl;
    pcl::toROSMsg(output_pcl, output);
    pub.publish(output);

}

int main (int argc, char **argv)
{
    ros::init (argc, argv, "ar_kinect");
    ros::NodeHandle n;

    listener = new tf::TransformListener();
    ros::Subscriber sub1 = n.subscribe("/robot1/camera/depth/points", 1, cloud_cb1);
    ros::Subscriber sub2 = n.subscribe("/robot2/camera/depth/points", 1, cloud_cb2);

    pub = n.advertise<sensor_msgs::PointCloud2>("/merged_points", 1);
    ros::spin ();

    return 0;
}
