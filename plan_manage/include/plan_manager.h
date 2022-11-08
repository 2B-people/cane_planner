#ifndef __PLAN_MANAGE__
#define __PLAN_MANAGE__

#include <iostream>
#include <vector>

#include <Eigen/Eigen>
#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>

namespace cane_planner
{
    class CanePlannerManager
    {
    private:
        /* data */
        enum FSM_STATE { INIT, WAIT_TARGET, GEN_NEW_TRAJ, REPLAN_TRAJ, EXEC_TRAJ, REPLAN_NEW };
        enum TARGET_TYPE { MANUAL_TARGET = 1, PRESET_TARGET = 2, REFENCE_PATH = 3 };

        // The node handle
        ros::NodeHandle node_;
        ros::Timer exec_timer_;
        ros::Subscriber map_sub_, odom_sub_, waypoint_sub_;

        void execFSMCallback(const ros::TimerEvent &e);
        void waypointCallback(const nav_msgs::PathConstPtr &msg);
        void odometryCallback(const nav_msgs::OdometryConstPtr &msg);
        // void MapCallback(const nav_msgs::OccupancyGrid::Ptr map);

    public:
        CanePlannerManager(int planner_);
        ~CanePlannerManager();
        void init(ros::NodeHandle &nh);
    };

} // namespace cane_planner

#endif