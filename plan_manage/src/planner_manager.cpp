#include <plan_manager.h>
#include <sstream>

namespace cane_planner
{

    PlannerManager::~PlannerManager()
    {
    }

    void PlannerManager::init(ros::NodeHandle &nh)
    {
        // init FSM
        exec_state_ = FSM_STATE::INIT;
        have_odom_ = false;
        have_target_ = false;

        // init esdf_map and collision
        sdf_map_.reset(new fast_planner::SDFMap);
        sdf_map_->initMap(nh);
        collision_.reset(new CollisionDetection);
        collision_->init(nh);
        collision_->setMap(sdf_map_);

        // init planner
        if (planner_ == 1)
        {
            ROS_WARN("choose Astar planer");
            astar_finder_.reset(new Astar);
            astar_finder_->setParam(nh);
            astar_finder_->setCollision(collision_);
            astar_finder_->init();
        }
        else if (planner_ == 2)
        {
            ROS_WARN("choose kinodynamic planer");
            kin_finder_.reset(new KinodynamicAstar);
            kin_finder_->setParam(nh);
            kin_finder_->setCollision(collision_);
            kin_finder_->init();
        }
        else
        {
            ROS_ERROR("Not choose planner");
        }

        // callback
        if (simulation_)
        {
            goal_sub_ =
                nh.subscribe("/move_base_simple/goal", 1, &PlannerManager::goalCallback, this); //接收目标的topic
            start_sub_ =
                nh.subscribe("/initialpose", 1, &PlannerManager::startCallback, this); //接收始点的topic
        }
        exec_timer_ =
            nh.createTimer(ros::Duration(0.01), &PlannerManager::execFSMCallback, this);
        waypoint_sub_ =
            nh.subscribe("/waypoint_generator/waypoints", 1, &PlannerManager::waypointCallback, this);
        odom_sub_ =
            nh.subscribe("/odom_world", 1, &PlannerManager::odometryCallback, this);

        // visial
        traj_pub_ = nh.advertise<visualization_msgs::Marker>("/planning_vis/trajectory", 20);
    }

    void PlannerManager::goalCallback(const geometry_msgs::PoseStamped::ConstPtr &goal)
    {
        end_pt_(0) = goal->pose.position.x;
        end_pt_(1) = goal->pose.position.y;
        ROS_INFO("set end pos is: %lf and %lf", end_pt_(0), end_pt_(1));
        have_target_ = true;
    }

    void PlannerManager::startCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &start)
    {
        odom_pos_(0) = start->pose.pose.position.x;
        odom_pos_(1) = start->pose.pose.position.y;
        ROS_INFO("set start pos is:%lf and %lf", odom_pos_(0), odom_pos_(1));
        have_odom_ = true;
    }

    void PlannerManager::waypointCallback(const nav_msgs::PathConstPtr &msg)
    {
        if (msg->poses[0].pose.position.z < -0.1)
            return;
        end_pt_ << msg->poses[0].pose.position.x, msg->poses[0].pose.position.y;

        have_target_ = true;
    }

    void PlannerManager::odometryCallback(const nav_msgs::OdometryConstPtr &msg)
    {
        odom_pos_(0) = msg->pose.pose.position.x;
        odom_pos_(1) = msg->pose.pose.position.y;
        odom_pos_(2) = msg->pose.pose.position.z;

        odom_vel_(0) = msg->twist.twist.linear.x;
        odom_vel_(1) = msg->twist.twist.linear.y;
        odom_vel_(2) = msg->twist.twist.linear.z;

        have_odom_ = true;
    }

    void PlannerManager::changeFSMExecState(FSM_STATE new_state)
    {
        string state_str[4] = {"INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "EXEC_TRAJ"};
        int pre_s = int(exec_state_);
        exec_state_ = new_state;
        cout << "[now]: from " + state_str[pre_s] + " to " + state_str[int(new_state)] << endl;
    }

    void PlannerManager::execFSMCallback(const ros::TimerEvent &e)
    {
        static int fsm_num = 0;
        fsm_num++;
        if (fsm_num == 100)
        {
            if (!have_odom_)
                ROS_WARN("no odom.");
            if (!have_target_)
                ROS_WARN("wait for goal.");
            fsm_num = 0;
        }

        switch (exec_state_)
        {
        case INIT:
        {
            if (!have_odom_)
                return;
            changeFSMExecState(WAIT_TARGET);
            exec_state_ = WAIT_TARGET;
            break;
        }
        case WAIT_TARGET:
        {
            if (!have_target_)
                return;
            changeFSMExecState(GEN_NEW_TRAJ);
            break;
        }
        case GEN_NEW_TRAJ:
        {
            // TODO:这里先用Odom当作start_pt;
            start_pt_(0) = odom_pos_(0);
            start_pt_(1) = odom_pos_(1);
            bool success = false;
            if (planner_ == 1)
                success = callAstarPlan();
            else if (planner_ == 2)
                success = callKinodynamicAstarPlan();
            if (success)
                changeFSMExecState(EXEC_TRAJ);
            else
                changeFSMExecState(GEN_NEW_TRAJ);
            break;
        }
        case EXEC_TRAJ:
        {
            // visiual
            displayPath();
            have_target_ = false;
            changeFSMExecState(WAIT_TARGET);
            break;
        }
        }
        return;
    }

    bool PlannerManager::callAstarPlan()
    {
        astar_finder_->reset();
        bool plan_success = astar_finder_->search(start_pt_, end_pt_);
        return plan_success;
    }

    bool PlannerManager::callKinodynamicAstarPlan()
    {
        kin_finder_->reset();
        // todo
        bool plan_success = false;
        return plan_success;
    }

    void PlannerManager::displayPath()
    {
        visualization_msgs::Marker mk;
        mk.header.frame_id = "world";
        mk.header.stamp = ros::Time::now();
        mk.type = visualization_msgs::Marker::SPHERE_LIST;
        mk.action = visualization_msgs::Marker::DELETE;
        mk.id = 0;

        mk.action = visualization_msgs::Marker::ADD;
        mk.pose.orientation.x = 0.0;
        mk.pose.orientation.y = 0.0;
        mk.pose.orientation.z = 0.0;
        mk.pose.orientation.w = 1.0;

        mk.color.r = 1.0;
        mk.color.g = 0.0;
        mk.color.b = 0.0;
        mk.color.a = 1;

        mk.scale.x = 0.1;
        mk.scale.y = 0.1;
        mk.scale.z = 0.1;

        geometry_msgs::Point pt;
        vector<Eigen::Vector2d> list;
        list = astar_finder_->getPath();
        for (int i = 0; i < int(list.size()); i++)
        {
            pt.x = list[i](0);
            pt.y = list[i](1);
            pt.z = 1.2;
            mk.points.push_back(pt);
        }

        traj_pub_.publish(mk);
        ros::Duration(0.001).sleep();
    }

} // namespace cane_planner
