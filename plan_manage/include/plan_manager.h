#ifndef __PLAN_MANAGE__
#define __PLAN_MANAGE__

#include <iostream>
#include <vector>

#include <Eigen/Eigen>
#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

// #include <nav_msgs/OccupancyGrid.h>

#include <path_searching/astar.h>
#include <path_searching/kinodynamic_astar.h>
#include <plan_env/collision_detection.h>

namespace cane_planner
{
    class PlannerManager
    {
    private:
        /* data */
        enum FSM_STATE
        {
            INIT,
            WAIT_TARGET,
            GEN_NEW_TRAJ,
            EXEC_TRAJ
        };
        /*---------- data -----------*/
        fast_planner::SDFMap::Ptr sdf_map_;
        CollisionDetection::Ptr collision_;

        unique_ptr<Astar> astar_finder_;
        unique_ptr<KinodynamicAstar> kin_finder_;

        int planner_;
        FSM_STATE exec_state_;
        bool have_odom_, have_target_;
        bool simulation_;

        Eigen::Vector3d odom_pos_, odom_vel_;

        Eigen::Vector2d start_pt_; // start state
        Eigen::Vector2d end_pt_;   // target state

        /*---------- Ros utils -----------*/
        ros::Timer exec_timer_;
        ros::Subscriber odom_sub_, waypoint_sub_;
        ros::Subscriber goal_sub_, start_sub_;

        /*---------- helper function -----------*/
        bool callAstarPlan();
        bool callKinodynamicAstarPlan();

        void changeFSMExecState(FSM_STATE new_state);

        /*---------- ROS function -----------*/
        void execFSMCallback(const ros::TimerEvent &e);
        void waypointCallback(const nav_msgs::PathConstPtr &msg);
        void odometryCallback(const nav_msgs::OdometryConstPtr &msg);
        void startCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &start);
        void goalCallback(const geometry_msgs::PoseStamped::ConstPtr &goal);
        // void MapCallback(const nav_msgs::OccupancyGrid::Ptr map);

    public:
        PlannerManager(int planner, bool simulation)
        {
            planner_ = planner;
            simulation_ = simulation;
        }
        ~PlannerManager();
        void init(ros::NodeHandle &nh);
    };

} // namespace cane_planner

#endif