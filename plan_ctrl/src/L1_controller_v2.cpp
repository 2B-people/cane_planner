/*
Copyright (c) 2017, ChanYuan KUO, YoRu LU,
latest editor: HaoChih, LIN
All rights reserved. (Hypha ROS Workshop)

This file is part of hypha_racecar package.

hypha_racecar is free software: you can redistribute it and/or modify
it under the terms of the GNU LESSER GENERAL PUBLIC LICENSE as published
by the Free Software Foundation, either version 3 of the License, or
any later version.

hypha_racecar is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU LESSER GENERAL PUBLIC LICENSE for more details.

You should have received a copy of the GNU LESSER GENERAL PUBLIC LICENSE
along with hypha_racecar.  If not, see <http://www.gnu.org/licenses/>.
*/

#include <iostream>
#include <string>

#include "ros/ros.h"
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>
#include <serial/serial.h>

#include <Eigen/Eigen>

#define PI 3.14159265358979

/********************/
/* CLASS DEFINITION */
/********************/
class L1Controller
{
public:
    L1Controller();
    void initMarker();
    bool isForwardWayPt(const geometry_msgs::Point &wayPt, const geometry_msgs::Pose &carPose);
    bool isWayPtAwayFromLfwDist(const geometry_msgs::Point &wayPt, const geometry_msgs::Point &car_pos);
    double getYawFromPose(const geometry_msgs::Pose &carPose);
    double getEta(const geometry_msgs::Pose &carPose);
    double getCar2GoalDist();
    double getL1Distance(const double &_Vcmd);
    double getSteeringAngle(double eta);
    double getGasInput(const float &current_v);
    geometry_msgs::Point get_odom_car2WayPtVec(const geometry_msgs::Pose &carPose);

private:
    ros::NodeHandle n_;
    ros::Subscriber odom_sub, path_sub, goal_sub, way_sub;
    ros::Publisher pub_, marker_pub;
    ros::Timer timer1, timer2;
    tf::TransformListener tf_listener;

    visualization_msgs::Marker points, line_strip, goal_circle;
    geometry_msgs::Twist cmd_vel;
    geometry_msgs::Point odom_goal_pos;
    nav_msgs::Odometry odom;
    nav_msgs::Path map_path, odom_path;

    // serial
    serial::Serial ser_;
    bool use_ser_flag_;
    int plan_;

    double L, Lfw, Lrv, Vcmd, lfw, lrv, steering, u, v;
    double Gas_gain, baseAngle, Angle_gain, goalRadius;
    int controller_freq, baseSpeed;
    bool foundForwardPt, goal_received, goal_reached;
    bool have_odom;

    void odomCB(const nav_msgs::Odometry::ConstPtr &odomMsg);
    void pathCB(const nav_msgs::Path::ConstPtr &pathMsg);
    void goalCB(const geometry_msgs::PoseStamped::ConstPtr &goalMsg);
    void waypointCB(const nav_msgs::PathConstPtr &msg);
    void goalReachingCB(const ros::TimerEvent &);
    void controlLoopCB(const ros::TimerEvent &);

}; // end of class

L1Controller::L1Controller()
{
    // Private parameters handler
    ros::NodeHandle pn("~");

    // Car parameter
    pn.param("L", L, 0.26);
    pn.param("Lrv", Lrv, 10.0);
    pn.param("Vcmd", Vcmd, 1.0);
    pn.param("lfw", lfw, 0.13);
    pn.param("lrv", lrv, 10.0);

    // Controller parameter
    pn.param("controller_freq", controller_freq, 20);
    pn.param("AngleGain", Angle_gain, -1.0);
    pn.param("GasGain", Gas_gain, 1.0);
    pn.param("baseSpeed", baseSpeed, 1470);
    pn.param("baseAngle", baseAngle, 90.0);
    pn.param("SerialUsing", use_ser_flag_, true);
    pn.param("/kin_replan_node/planner_node/planner", plan_, 1);

    // Publishers and Subscribers
    odom_sub = n_.subscribe("/odometry/filtered", 1, &L1Controller::odomCB, this);
    if (plan_ == 1)
    {
        path_sub = n_.subscribe("/astar/path", 1, &L1Controller::pathCB, this);
        ROS_WARN("using astar");
    }
    else if (plan_ == 2)
    {
        path_sub = n_.subscribe("/kin_astar/path", 1, &L1Controller::pathCB, this);
        ROS_WARN("using kin_astar");
    }
    goal_sub = n_.subscribe("/move_base_simple/goal", 1, &L1Controller::goalCB, this);
    way_sub = n_.subscribe("/waypoint_generator/waypoints", 1, &L1Controller::waypointCB, this);

    marker_pub = n_.advertise<visualization_msgs::Marker>("car_path", 10);
    pub_ = n_.advertise<geometry_msgs::Twist>("car/cmd_vel", 1);

    // Timer
    timer1 = n_.createTimer(ros::Duration((1.0) / controller_freq), &L1Controller::controlLoopCB, this);  // Duration(0.05) -> 20Hz
    timer2 = n_.createTimer(ros::Duration((0.5) / controller_freq), &L1Controller::goalReachingCB, this); // Duration(0.05) -> 20Hz

    std::string port("/dev/ttyUSB0");
    int baudrate = 115200;

    pn.param("port", port, port);
    pn.param("baudrate", baudrate, baudrate);

    // Serial
    if (use_ser_flag_)
    {
        ser_.setPort(port);
        ser_.setBaudrate(baudrate);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        ser_.setTimeout(to);
        ROS_WARN("---[param] serial set:%s, baudrate set:%d----", port.c_str(), baudrate);
        try
        {
            ser_.open();
        }
        catch (serial::IOException &e)
        {
            ROS_ERROR_STREAM("Unable to open port ");
        }
        if (ser_.isOpen())
        {
            ROS_INFO_STREAM("Serial Port initialized");
        }
        else
        {
            ROS_ERROR_STREAM("Serial Port fail");
        }
        std::string send_data = "z000\n";
        u_char send_data_char[send_data.size()];
        for (size_t i = 0; i < send_data.size(); i++)
            send_data_char[i] = send_data.c_str()[i];
        ser_.write(send_data_char, send_data.size());
    }
    else
    {
        ROS_WARN("-----NO Serial!------ ");
    }

    // Init variables
    Lfw = goalRadius = getL1Distance(Vcmd);
    foundForwardPt = false;
    goal_received = false;
    goal_reached = false;
    have_odom = false;
    cmd_vel.linear.x = 1500; // 1500 for stop
    cmd_vel.angular.z = baseAngle;

    // Show info
    ROS_INFO("[param] baseSpeed: %d", baseSpeed);
    ROS_INFO("[param] baseAngle: %f", baseAngle);
    ROS_INFO("[param] AngleGain: %f", Angle_gain);
    ROS_INFO("[param] Vcmd: %f", Vcmd);
    ROS_INFO("[param] Lfw: %f", Lfw);

    // Visualization Marker Settings
    initMarker();
}

void L1Controller::initMarker()
{
    points.header.frame_id = line_strip.header.frame_id = goal_circle.header.frame_id = "world";
    points.ns = line_strip.ns = goal_circle.ns = "Markers";
    points.action = line_strip.action = goal_circle.action = visualization_msgs::Marker::ADD;
    points.pose.orientation.w = line_strip.pose.orientation.w = goal_circle.pose.orientation.w = 1.0;
    points.id = 0;
    line_strip.id = 1;
    goal_circle.id = 2;

    points.type = visualization_msgs::Marker::POINTS;
    line_strip.type = visualization_msgs::Marker::LINE_STRIP;
    goal_circle.type = visualization_msgs::Marker::CYLINDER;
    // POINTS markers use x and y scale for width/height respectively
    points.scale.x = 0.2;
    points.scale.y = 0.2;

    // LINE_STRIP markers use only the x component of scale, for the line width
    line_strip.scale.x = 0.1;

    goal_circle.scale.x = goalRadius;
    goal_circle.scale.y = goalRadius;
    goal_circle.scale.z = 0.1;

    // Points are green
    points.color.g = 1.0f;
    points.color.a = 1.0;

    // Line strip is blue
    line_strip.color.b = 1.0;
    line_strip.color.a = 1.0;

    // goal_circle is yellow
    goal_circle.color.r = 1.0;
    goal_circle.color.g = 1.0;
    goal_circle.color.b = 0.0;
    goal_circle.color.a = 0.5;
}

void L1Controller::odomCB(const nav_msgs::Odometry::ConstPtr &odomMsg)
{
    odom = *odomMsg;
    have_odom = true;
}

void L1Controller::pathCB(const nav_msgs::Path::ConstPtr &pathMsg)
{
    map_path = *pathMsg;
}

void L1Controller::waypointCB(const nav_msgs::PathConstPtr &msg)
{
    geometry_msgs::PoseStamped odom_goal;
    // tf_listener.transformPose("world", ros::Time(0), *goalMsg, "world", odom_goal);
    // odom_goal_pos = odom_goal.pose.position;
    odom_goal_pos = msg->poses[0].pose.position;
    goal_received = true;
    goal_reached = false;

    /*Draw Goal on RVIZ*/
    goal_circle.pose = odom_goal.pose;
    marker_pub.publish(goal_circle);
}

void L1Controller::goalCB(const geometry_msgs::PoseStamped::ConstPtr &goalMsg)
{
    // try
    // {
    geometry_msgs::PoseStamped odom_goal;
    // tf_listener.transformPose("world", ros::Time(0), *goalMsg, "world", odom_goal);
    // odom_goal_pos = odom_goal.pose.position;
    odom_goal_pos = goalMsg->pose.position;
    goal_received = true;
    goal_reached = false;

    /*Draw Goal on RVIZ*/
    goal_circle.pose = odom_goal.pose;
    marker_pub.publish(goal_circle);
    // }
    // catch (tf::TransformException &ex)
    // {
    //     ROS_ERROR("%s", ex.what());
    //     ros::Duration(1.0).sleep();
    // }
}

double L1Controller::getYawFromPose(const geometry_msgs::Pose &carPose)
{
    Eigen::Quaterniond ori;
    ori.x() = carPose.orientation.x;
    ori.y() = carPose.orientation.y;
    ori.z() = carPose.orientation.z;
    ori.w() = carPose.orientation.w;

    // have bug
    // Eigen::Matrix3d oRx = ori.toRotationMatrix();
    // double yaw_t = 0, pitch = -M_PI / 2, roll = M_PI / 2;
    // Eigen::Matrix3d Rx;
    // Rx = Eigen::AngleAxisd(yaw_t, Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());
    // oRx = oRx * Rx;
    // Eigen::Vector3d ea = oRx.eulerAngles(2, 1, 0);
    // double tmp, yaw;
    // tf::Quaternion q(x, y, z, w);
    // tf::Matrix3x3 quaternion(q);
    // quaternion.getRPY(tmp, tmp, yaw);
    double yaw = 0.0;
    Eigen::Vector3d rot_x = ori.toRotationMatrix().block(0, 0, 3, 1);
    yaw = atan2(rot_x(1), rot_x(0));
    return yaw;
}

bool L1Controller::isForwardWayPt(const geometry_msgs::Point &wayPt, const geometry_msgs::Pose &carPose)
{
    float car2wayPt_x = wayPt.x - carPose.position.x;
    float car2wayPt_y = wayPt.y - carPose.position.y;
    // 这里用的是carpose的yaw，我的代码中是以cane为坐标系的
    double car_theta = getYawFromPose(carPose);
    car_theta = car_theta;
    // 正x方向为车前向
    float car_car2wayPt_x = cos(car_theta) * car2wayPt_x + sin(car_theta) * car2wayPt_y;
    float car_car2wayPt_y = -sin(car_theta) * car2wayPt_x + cos(car_theta) * car2wayPt_y;

    if (car_car2wayPt_x > 0 || car_car2wayPt_y > 0) /*is Forward WayPt*/
    {
        ROS_WARN("Forward WayPt");
        return true;
    }
    else
        return false;
}

bool L1Controller::isWayPtAwayFromLfwDist(const geometry_msgs::Point &wayPt, const geometry_msgs::Point &car_pos)
{
    double dx = wayPt.x - car_pos.x;
    double dy = wayPt.y - car_pos.y;
    double dist = sqrt(dx * dx + dy * dy);

    if (dist >= Lfw && dist <= 1.0)
        return true;
    else
        return false;
}

geometry_msgs::Point L1Controller::get_odom_car2WayPtVec(const geometry_msgs::Pose &carPose)
{
    geometry_msgs::Point carPose_pos = carPose.position;
    double carPose_yaw = getYawFromPose(carPose);
    geometry_msgs::Point forwardPt;
    geometry_msgs::Point odom_car2WayPtVec;
    foundForwardPt = false;

    if (!goal_reached)
    {
        for (size_t i = 0; i < map_path.poses.size(); i++)
        {
            // 取规划的前向点
            //  bug:有些时候无法取到有效的前向点
            geometry_msgs::PoseStamped map_path_pose = map_path.poses[i];
            geometry_msgs::PoseStamped odom_path_pose;

            try
            {
                tf_listener.transformPose("world", ros::Time(0), map_path_pose, "world", odom_path_pose);
                geometry_msgs::Point odom_path_wayPt = odom_path_pose.pose.position;
                // 前向点判读
                bool _isForwardWayPt = isForwardWayPt(odom_path_wayPt, carPose);

                if (_isForwardWayPt)
                {
                    // 前向点Lfw判读
                    bool _isWayPtAwayFromLfwDist = isWayPtAwayFromLfwDist(odom_path_wayPt, carPose_pos);
                    if (_isWayPtAwayFromLfwDist)
                    {
                        forwardPt = odom_path_wayPt;
                        foundForwardPt = true;
                        break;
                    }
                }
            }
            catch (tf::TransformException &ex)
            {
                ROS_ERROR("%s", ex.what());
            }
        }
    }
    else if (goal_reached)
    {
        forwardPt = odom_goal_pos;
        foundForwardPt = false;
        ROS_WARN("goal REACHED!");
    }

    /*Visualized Target Point on RVIZ*/
    /*Clear former target point Marker*/
    points.points.clear();
    line_strip.points.clear();

    if (foundForwardPt && !goal_reached)
    {
        points.points.push_back(carPose_pos);
        points.points.push_back(forwardPt);
        line_strip.points.push_back(carPose_pos);
        line_strip.points.push_back(forwardPt);
    }

    marker_pub.publish(points);
    marker_pub.publish(line_strip);

    odom_car2WayPtVec.x = cos(carPose_yaw) * (forwardPt.x - carPose_pos.x) + sin(carPose_yaw) * (forwardPt.y - carPose_pos.y);
    odom_car2WayPtVec.y = -sin(carPose_yaw) * (forwardPt.x - carPose_pos.x) + cos(carPose_yaw) * (forwardPt.y - carPose_pos.y);
    return odom_car2WayPtVec;
}

double L1Controller::getEta(const geometry_msgs::Pose &carPose)
{
    geometry_msgs::Point odom_car2WayPtVec = get_odom_car2WayPtVec(carPose);

    double eta = atan2(odom_car2WayPtVec.y, odom_car2WayPtVec.x);
    return eta;
}

double L1Controller::getCar2GoalDist()
{
    geometry_msgs::PoseStamped pose_cam;
    geometry_msgs::PoseStamped pose_world;
    if (have_odom)
    {
        pose_cam.header = odom.header;
        pose_cam.pose = odom.pose.pose;
        tf_listener.transformPose("world", pose_cam, pose_world);
    }
    geometry_msgs::Point car_pose = pose_world.pose.position;

    double car2goal_x = odom_goal_pos.x - car_pose.x;
    double car2goal_y = odom_goal_pos.y - car_pose.y;

    double dist2goal = sqrt(car2goal_x * car2goal_x + car2goal_y * car2goal_y);

    return dist2goal;
}

double L1Controller::getL1Distance(const double &_Vcmd)
{
    double L1 = 0;
    if (_Vcmd < 1.34)
        L1 = 3 / 6.0;
    else if (_Vcmd > 1.34 && _Vcmd < 5.36)
        L1 = _Vcmd * 2.24 / 3.0;
    else
        L1 = 12 / 3.0;
    return L1;
}

double L1Controller::getSteeringAngle(double eta)
{
    double steeringAnge = -atan2((L * sin(eta)), (Lfw / 2 + lfw * cos(eta))) * (180.0 / PI);
    // ROS_INFO("Steering Angle = %.2f", steeringAnge);
    return steeringAnge;
}

double L1Controller::getGasInput(const float &current_v)
{
    double u = (Vcmd - current_v) * Gas_gain;
    // ROS_INFO("velocity = %.2f\tu = %.2f",current_v, u);
    return u;
}

void L1Controller::goalReachingCB(const ros::TimerEvent &)
{
    if (goal_received)
    {
        double car2goal_dist = getCar2GoalDist();
        if (car2goal_dist < goalRadius)
        {
            goal_reached = true;
            goal_received = false;
            // if (use_ser_flag_)
            // {
            //     std::string send_data = "z000\n";
            //     u_char send_data_char[send_data.size()];
            //     for (size_t i = 0; i < send_data.size(); i++)
            //         send_data_char[i] = send_data.c_str()[i];
            //     ser_.write(send_data_char, send_data.size());
            // }
            ROS_WARN("Goal Reached !");
        }
    }
    if (use_ser_flag_)
    {
        u_char recv_data[200];

        if (ser_.available())
        {
            // ROS_INFO_STREAM("Reading from serial port\n");
            // 保存串口数据至数值 recv_data[200]
            ser_.read(recv_data, ser_.available());
            // ROS_INFO("%s", recv_data);
        }
    }
}

void L1Controller::controlLoopCB(const ros::TimerEvent &)
{
    geometry_msgs::Pose carPose;
    geometry_msgs::PoseStamped pose_cam;
    geometry_msgs::PoseStamped pose_world;
    if (have_odom)
    {
        pose_cam.header = odom.header;
        pose_cam.pose = odom.pose.pose;
        tf_listener.transformPose("world", pose_cam, pose_world);

        tf::StampedTransform trans;
        try
        {
            tf_listener.lookupTransform("/world", "/cane_base", ros::Time(), trans);
        }
        catch (tf::TransformException &ex)
        {
            ROS_ERROR("%s", ex.what());
            return;
        }
        auto cane_Q = trans.getRotation();
        carPose.orientation.x = cane_Q.getX();
        carPose.orientation.y = cane_Q.getY();
        carPose.orientation.x = cane_Q.getZ();
        carPose.orientation.w = cane_Q.getW();
        carPose.position = pose_world.pose.position;
    }

    // geometry_msgs::Twist carVel = odom.twist.twist;
    cmd_vel.linear.x = 1500;
    cmd_vel.angular.z = baseAngle;

    if (goal_received)
    {
        /*Estimate Steering Angle*/
        double eta = getEta(carPose) + 1.57;
        if (foundForwardPt)
        {

            cmd_vel.angular.z = getSteeringAngle(eta) * Angle_gain;
            ROS_WARN("\nEstimate Steering Angle angle = %f", eta);
            ROS_INFO("\nSteering angle = %d", (int)(cmd_vel.angular.z) * 100);

            /*Estimate Gas Input*/
            if (!goal_reached)
            {
                // double u = getGasInput(carVel.linear.x);
                // cmd_vel.linear.x = baseSpeed - u;
                cmd_vel.linear.x = baseSpeed;
                if (use_ser_flag_)
                {
                    std::string send_data = "z" + std::to_string((int)(cmd_vel.angular.z * 100)) + "\n";
                    u_char send_data_char[send_data.size()];
                    for (size_t i = 0; i < send_data.size(); i++)
                        send_data_char[i] = send_data.c_str()[i];
                    ser_.write(send_data_char, send_data.size());
                }
                else
                {
                    pub_.publish(cmd_vel);
                }
            }
        }
    }
}

/*****************/
/* MAIN FUNCTION */
/*****************/
int main(int argc, char **argv)
{
    // Initiate ROS
    ros::init(argc, argv, "L1Controller_v2");
    L1Controller controller;
    ros::spin();
    return 0;
}
