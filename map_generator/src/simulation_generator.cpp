#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>

using namespace std;
string file_name;
ros::Publisher odom_pub,pose_pub;
nav_msgs::Odometry odom;

void testCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &start)
{
  odom.header.frame_id = "world";
  odom.pose = start->pose;
  ROS_WARN("get start");
}

void cmdCallback(const geometry_msgs::Twist::ConstPtr &twist)
{
  static ros::Time current_time = ros::Time::now();
  ros::Time now_time = ros::Time::now();
  double T = now_time.toSec() - current_time.toSec();
  ROS_INFO("time change is %lf",T);

  // odom 
  // TODO:odom change

  
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "map_recorder");
  ros::NodeHandle node;

  ros::Publisher cloud_pub =
      node.advertise<sensor_msgs::PointCloud2>("/Simulation_generator/global_cloud", 10, true);
  pose_pub =
      node.advertise<geometry_msgs::PoseStamped>("/simulation_generator/pose", 10, true);
  odom_pub =
      node.advertise<nav_msgs::Odometry>("/Simulation_generator/odom", 10, true);

  ros::Subscriber start_sub = node.subscribe("/initialpose", 10, testCallback);
  ros::Subscriber cmd_sub = node.subscribe("/cmd_vel",10,cmdCallback);


  file_name = argv[1];

  // ros::Duration(1.0).sleep();

  /* load cloud from pcd */
  pcl::PointCloud<pcl::PointXYZ> cloud;
  int status = pcl::io::loadPCDFile<pcl::PointXYZ>(file_name, cloud);
  if (status == -1)
  {
    cout << "can't read file." << endl;
    return -1;
  }
  // init odom
  odom.header.frame_id = "world";
  odom.header.stamp = ros::Time::now();
  odom.pose.pose.position.x = 0.0;
  odom.pose.pose.position.y = 0.0;
  odom.pose.pose.position.z = 0.0;
  odom.pose.pose.orientation.w = 1.0;
  odom.pose.pose.orientation.x = 0.0;
  odom.pose.pose.orientation.y = 0.0;
  odom.pose.pose.orientation.z = 0.0;

  // Find range of map
  Eigen::Vector2d mmin(0, 0), mmax(0, 0);
  for (auto pt : cloud)
  {
    mmin[0] = min(mmin[0], double(pt.x));
    mmin[1] = min(mmin[1], double(pt.y));
    mmax[0] = max(mmax[0], double(pt.x));
    mmax[1] = max(mmax[1], double(pt.y));
  }
  // Add ground
  for (double x = mmin[0]; x <= mmax[0]; x += 0.1)
    for (double y = mmin[1]; y <= mmax[1]; y += 0.1)
    {
      cloud.push_back(pcl::PointXYZ(x, y, 0));
    }

  sensor_msgs::PointCloud2 msg;
  pcl::toROSMsg(cloud, msg);

  int count = 0;
  while (1)
  {
    ros::Duration(0.1).sleep();
    msg.header.frame_id = "world";
    msg.header.stamp = ros::Time::now();
    cloud_pub.publish(msg);
    if (count++ > 10)
    {
      break;
    }
  }

  while (ros::ok())
  {
    ros::Duration(0.05).sleep();

    odom.header.stamp = ros::Time::now();
    odom_pub.publish(odom);

    ros::spinOnce();
  }

  return 0;
}