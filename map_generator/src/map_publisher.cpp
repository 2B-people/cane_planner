#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>

using namespace std;
string file_name;

int main(int argc, char** argv) {
  ros::init(argc, argv, "map_recorder");
  ros::NodeHandle node;

  ros::Publisher cloud_pub =
      node.advertise<sensor_msgs::PointCloud2>("/map_generator/global_cloud", 10, true);
  ros::Publisher pose_pub = 
      node.advertise<geometry_msgs::PoseStamped>("/map_generator/pose", 10, true);
  ros::Publisher odom_pub =
      node.advertise<nav_msgs::Odometry>("/simulation_generator/odom", 10, true);
  file_name = argv[1];

  ros::Duration(1.0).sleep();

  /* load cloud from pcd */
  pcl::PointCloud<pcl::PointXYZ> cloud;
  int status = pcl::io::loadPCDFile<pcl::PointXYZ>(file_name, cloud);
  if (status == -1) {
    cout << "can't read file." << endl;
    return -1;
  }

  // // Transform map
  // for (int i = 0; i < cloud.points.size(); ++i)
  // {
  //   auto pt = cloud.points[i];
  //   pcl::PointXYZ pr;
  //   pr.x = pt.x;
  //   pr.y = -pt.z; 
  //   pr.z = pt.y;
  //   cloud.points[i] = pr;
  // }

  // Find range of map
  // Eigen::Vector2d mmin(0, 0), mmax(0, 0);
  // for (auto pt : cloud) {
  //   mmin[0] = min(mmin[0], double(pt.x));
  //   mmin[1] = min(mmin[1], double(pt.y));
  //   mmax[0] = max(mmax[0], double(pt.x));
  //   mmax[1] = max(mmax[1], double(pt.y));
  // }

  // // Add ground
  // for (double x = mmin[0]; x <= mmax[0]; x += 0.1)
  //   for (double y = mmin[1]; y <= mmax[1]; y += 0.1) {
  //     cloud.push_back(pcl::PointXYZ(x, y, -0.5));
  //   }

  // cout << "Publishing map..." << endl;

  // cloud
  sensor_msgs::PointCloud2 msg;
  msg.header.frame_id = "world";
  pcl::toROSMsg(cloud, msg);

  // pose
  geometry_msgs::PoseStamped pose;
  pose.header.frame_id = "world";
  pose.pose.position.x = 0.0;
  pose.pose.position.y = 0.0;
  pose.pose.position.z = 0.0;
  pose.pose.orientation.w = 1.0;
  pose.pose.orientation.x = 0.0;
  pose.pose.orientation.y = 0.0;
  pose.pose.orientation.z = 0.0; 

  // odom
  nav_msgs::Odometry odom;
  odom.header.frame_id = "world";
  odom.pose.pose.position.x = 0.0;
  odom.pose.pose.position.y = 0.0;
  odom.pose.pose.position.z = 0.0;
  odom.pose.pose.orientation.w = 1.0;
  odom.pose.pose.orientation.x = 0.0;
  odom.pose.pose.orientation.y = 0.0;
  odom.pose.pose.orientation.z = 0.0; 

  
  int count = 0;
  while (ros::ok()) {
    ros::Duration(0.2).sleep();
    cloud_pub.publish(msg);
    pose_pub.publish(pose);
    odom_pub.publish(odom);
    ++count;
    if (count > 10) {
      break;
    }
  }

  cout << "finish publish map." << endl;

  return 0;
}