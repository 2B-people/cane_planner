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
        // init kin planner
        ROS_WARN(" Astar planer start");
        astar_finder_.reset(new Astar);
        astar_finder_->setParam(nh);
        astar_finder_->setCollision(collision_);
        astar_finder_->init();
        // init lfpc model
        lfpc_model_.reset(new LFPC);
        lfpc_model_->initializeModel(nh);
        // init astar planner
        ROS_WARN(" kinodynamic planer start");
        kin_finder_.reset(new KinodynamicAstar);
        kin_finder_->setParam(nh);
        kin_finder_->setCollision(collision_);
        kin_finder_->setModel(lfpc_model_);
        kin_finder_->init();
        // simulation kin plan vis astar plan
        if (simulation_)
        {
            goal_sub_ =
                nh.subscribe("/move_base_simple/goal", 1, &PlannerManager::goalCallback, this); // 接收目标的topic
            start_sub_ =
                nh.subscribe("/initialpose", 1, &PlannerManager::startCallback, this); // 接收始点的topic
        }
        else // replan
        {
            waypoint_sub_ =
                nh.subscribe("/waypoint_generator/waypoints", 1, &PlannerManager::waypointCallback, this);
            odom_sub_ =
                nh.subscribe("/odom_world", 1, &PlannerManager::odometryCallback, this);
        }
        // Timer
        exec_timer_ =
            nh.createTimer(ros::Duration(0.05), &PlannerManager::execFSMCallback, this);
        replan_timer_ =
            nh.createTimer(ros::Duration(0.1), &PlannerManager::checkCollisionCallback, this);
        // Visial
        astar_pub_ = nh.advertise<visualization_msgs::Marker>("/planning_vis/astar", 20);
        kin_path_pub_ = nh.advertise<visualization_msgs::Marker>("/planning_vis/kin_astar", 20);
        kin_foot_pub_ = nh.advertise<visualization_msgs::Marker>("/planning_vis/kin_foot", 20);
        // Path
        path_pub_ = nh.advertise<nav_msgs::Path>("/kin_astar/path", 20);
    }
    //------------------- simulation ---------------------
    // simulation callback goal
    void PlannerManager::goalCallback(const geometry_msgs::PoseStamped::ConstPtr &goal)
    {
        end_pt_(0) = goal->pose.position.x;
        end_pt_(1) = goal->pose.position.y;
        ROS_INFO("set end pos is: %lf and %lf", end_pt_(0), end_pt_(1));

        end_state_(0) = goal->pose.position.x;
        end_state_(1) = goal->pose.position.y;
        double yaw = QuatenionToYaw(goal->pose.orientation);
        end_state_(2) = yaw;
        ROS_INFO("goal yaw is: %lf", yaw);
        have_target_ = true;
    }
    // simulation callback start or odom
    void PlannerManager::startCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &start)
    {
        start_pt_(0) = start->pose.pose.position.x;
        start_pt_(1) = start->pose.pose.position.y;
        ROS_INFO("set start pos is:%lf and %lf", start_pt_(0), start_pt_(1));

        start_state_(0) = start->pose.pose.position.x;
        start_state_(1) = start->pose.pose.position.y;
        double yaw = QuatenionToYaw(start->pose.pose.orientation);
        start_state_(2) = yaw;
        cout << "yaw:" << yaw << endl;

        have_odom_ = true;
    }
    //------------------- real experience ---------------------
    // real experience callback waypoint or goal
    void PlannerManager::waypointCallback(const nav_msgs::PathConstPtr &msg)
    {
        if (msg->poses[0].pose.position.z < -0.1)
            return;
        end_pt_ << msg->poses[0].pose.position.x, msg->poses[0].pose.position.y;
        end_state_(0) = msg->poses[0].pose.position.x;
        end_state_(1) = msg->poses[0].pose.position.y;
        double yaw = QuatenionToYaw(msg->poses[0].pose.orientation);
        end_state_(2) = yaw;
        ROS_INFO("set end pos is: %lf and %lf", end_pt_(0), end_pt_(1));
        ROS_INFO("end yaw is: %lf", yaw);
        have_target_ = true;
    }
    // odomtry
    void PlannerManager::odometryCallback(const nav_msgs::OdometryConstPtr &msg)
    {
        odom_pos_(0) = msg->pose.pose.position.x;
        odom_pos_(1) = msg->pose.pose.position.y;
        odom_pos_(2) = msg->pose.pose.position.z;
        odom_vel_(0) = msg->twist.twist.linear.x;
        odom_vel_(1) = msg->twist.twist.linear.y;
        odom_vel_(2) = msg->twist.twist.linear.z;
        odom_ori_.x() = msg->pose.pose.orientation.x;
        odom_ori_.y() = msg->pose.pose.orientation.y;
        odom_ori_.z() = msg->pose.pose.orientation.z;
        odom_ori_.w() = msg->pose.pose.orientation.w;
        //! test yaw
        double yaw_test = QuatenionToYaw(msg->pose.pose.orientation);

        // odom and start set
        start_pt_(0) = odom_pos_(0);
        start_pt_(1) = odom_pos_(1);
        start_state_(0) = odom_pos_(0);
        start_state_(1) = odom_pos_(1);
        // TODO:vins' yaw have bug,need
        double yaw = QuatenionToYaw(odom_ori_);
        start_state_(2) = yaw;

        ROS_WARN("start_pt_ is %f and %f", start_pt_(0), start_pt_(1));
        ROS_WARN("odom_yaw is %f,change is %f", yaw_test, yaw);
        have_odom_ = true;
    }
    // ------------------------ FSM Callback --------------------------------
    void PlannerManager::execFSMCallback(const ros::TimerEvent &e)
    {
        static int fsm_num = 0;
        static bool success1 = false;
        static bool success2 = false;
        fsm_num++;
        if (fsm_num == 100)
        {
            if (!have_odom_)
                // ROS_WARN("no odom.");
                if (!have_target_)
                    // ROS_WARN("wait for goal.");
                    fsm_num = 0;
        }
        // FSM loop
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
            success1 = callAstarPlan();
            success2 = callKinodynamicAstarPlan();
            if (success1 || success2)
                changeFSMExecState(EXEC_TRAJ);
            else
                changeFSMExecState(REPLAN_TRAJ);
            break;
        }
        case REPLAN_TRAJ:
        {
            // replan just for kinplan
            success2 = callKinodynamicAstarPlan();
            if (success2)
                changeFSMExecState(EXEC_TRAJ);
            else
                changeFSMExecState(REPLAN_TRAJ);
            break;
        }
        case EXEC_TRAJ:
        {
            if (success1) // a star success
            {
                displayAstar();
            }
            if (success2) // kin star success
            {
                displayKinastar();
                publishKinodynamicAstarPath();
            }
            // simulation stop replan
            if (simulation_)
            {
                have_target_ = false;
                changeFSMExecState(WAIT_TARGET);
            }
            else
            {
                // real experience using odom judge stop replan
                if (abs(odom_pos_(0) - end_pt_(0)) <= 0.4 ||
                    abs(odom_pos_(1) - end_pt_(1)) <= 0.4)
                {
                    have_target_ = false;
                    changeFSMExecState(WAIT_TARGET);
                }
                else // replan
                {
                    changeFSMExecState(REPLAN_TRAJ);
                }
            }
            break;
        }
        }
        return;
    }
    // --------------------------- Collision replan ----------------------------
    void PlannerManager::checkCollisionCallback(const ros::TimerEvent &e)
    {
        // end pos is in Collision,change end pos in 0.5 range
        if (have_target_)
        {
            double dist = collision_->getCollisionDistance(end_pt_);
            if (dist <= 0.3)
            {
                /* try to find a max distance goal around */
                const double dr = 0.5, dtheta = 30;
                double new_x, new_y, new_z, max_dist = -1.0;
                Eigen::Vector3d goal(-1, -1, -1);
                for (double r = dr; r <= 5 * dr + 1e-3; r += dr)
                {
                    for (double theta = -90; theta <= 270; theta += dtheta)
                    {
                        new_x = end_pt_(0) + r * cos(theta / 57.3);
                        new_y = end_pt_(1) + r * sin(theta / 57.3);
                        new_z = 1.0;
                        Eigen::Vector2d new_pt(new_x, new_y);
                        dist = collision_->getCollisionDistance(new_pt);
                        if (dist > max_dist)
                        {
                            /* reset end_pt_ */
                            goal(0) = new_x;
                            goal(1) = new_y;
                            goal(2) = new_z;
                            max_dist = dist;
                        }
                    }
                }
                if (max_dist > 0.3)
                {
                    end_pt_ << goal(0), goal(1);
                    end_state_(0) = goal(0);
                    end_state_(1) = goal(1);
                    have_target_ = true;
                    if (exec_state_ == EXEC_TRAJ)
                    {
                        ROS_WARN("goal near collision,change end");
                        changeFSMExecState(REPLAN_TRAJ);
                    }
                }
                else
                {
                    have_target_ = false;
                    cout << "Goal near collision, stop." << endl;
                    changeFSMExecState(WAIT_TARGET);
                }
            }
        }
        // Collision replan
        if (exec_state_ == EXEC_TRAJ)
        {
            vector<Eigen::Vector3d> list;
            list = kin_finder_->getPath();
            for (size_t i = 0; i < list.size(); i++)
            {
                Eigen::Vector2d temp(list[i](0), list[i](1));
                double dist = collision_->getCollisionDistance(temp);
                // todo 这里的距离可以修改
                if (dist < 0.3)
                {
                    ROS_WARN("current traj in collision.");
                    changeFSMExecState(REPLAN_TRAJ);
                }
            }
        }
        return;
    }

    // ------------------------- helper function -------------------------------------
    void PlannerManager::changeFSMExecState(FSM_STATE new_state)
    {
        string state_str[5] = {"INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "EXEC_TRAJ", "REPLAN_TRAJ"};
        int pre_s = int(exec_state_);
        exec_state_ = new_state;
        cout << "[now]: from " + state_str[pre_s] + " to " + state_str[int(new_state)] << endl;
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
        Eigen::Vector3d input;
        // double vx, vy;
        // vx = 0.5 * sin(start_state_(2));
        // vy = 0.5 * cos(start_state_(2));
        // input << 0.0, vx, 0.0, vy;
        input << 0.0, 0.0, start_state_(2);
        bool plan_success = kin_finder_->search(start_state_, input, end_state_);
        return plan_success;
    }
    // publish traj to L1-control
    void PlannerManager::publishKinodynamicAstarPath()
    {
        vector<Eigen::Vector3d> list;
        list = kin_finder_->getPath();
        nav_msgs::Path path;
        path.header.frame_id = "world";
        path.header.stamp = ros::Time::now();
        for (size_t i = 0; i < list.size(); i++)
        {
            geometry_msgs::PoseStamped this_pose_stamped;
            this_pose_stamped.pose.position.x = list[i](0);
            this_pose_stamped.pose.position.y = list[i](1);
            this_pose_stamped.pose.position.z = -0.4;
            this_pose_stamped.pose.orientation.x = 0.0;
            this_pose_stamped.pose.orientation.y = 0.0;
            this_pose_stamped.pose.orientation.z = 1.0;
            this_pose_stamped.pose.orientation.w = 1.0;
            this_pose_stamped.header.frame_id = "world";
            this_pose_stamped.header.stamp = ros::Time::now();
            path.poses.push_back(this_pose_stamped);
        }
        path_pub_.publish(path);
    }
    // visial
    void PlannerManager::displayAstar()
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
            pt.z = 1 - 0.4;
            mk.points.push_back(pt);
        }

        astar_pub_.publish(mk);
        ros::Duration(0.001).sleep();
    }

    void PlannerManager::displayKinastar()
    {
        // marker set
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
        mk.color.r = 0.0;
        mk.color.g = 0.0;
        mk.color.b = 1.0;
        mk.color.a = 1;
        mk.scale.x = 0.1;
        mk.scale.y = 0.1;
        mk.scale.z = 0.1;
        // give point
        geometry_msgs::Point pt;
        vector<Eigen::Vector3d> list;
        list = kin_finder_->getPath();
        for (int i = 0; i < int(list.size()); i++)
        {
            pt.x = list[i](0);
            pt.y = list[i](1);
            pt.z = 1 - 0.4;
            mk.points.push_back(pt);
        }
        // publish traj
        kin_path_pub_.publish(mk);

        // set feet pos publisher
        mk.points.clear();
        mk.color.r = 0.0;
        mk.color.g = 1.0;
        mk.color.b = 0.0;
        list.clear();
        list = kin_finder_->getFeetPos();
        for (int i = 0; i < int(list.size()); i++)
        {
            pt.x = list[i](0);
            pt.y = list[i](1);
            pt.z = 0;
            mk.points.push_back(pt);
        }
        // publish feet
        kin_foot_pub_.publish(mk);

        ros::Duration(0.001).sleep();
    }
    // calculate YAW by different function
    double PlannerManager::QuatenionToYaw(geometry_msgs::Quaternion ori)
    {
        tf::Quaternion quat;
        tf::quaternionMsgToTF(ori, quat);
        double roll, pitch, yaw;
        tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
        return yaw;
    }
    // TODO这里待验证正确性
    double PlannerManager::QuatenionToYaw(Eigen::Quaterniond ori)
    {
        Eigen::Matrix3d oRx = ori.toRotationMatrix();
        // roll world to body is 
        double yaw = 0, pitch = 0, roll = M_PI / 2;
        Eigen::Matrix3d Rx;
        Rx = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());
        oRx = oRx * Rx;
        Eigen::Vector3d ea = oRx.eulerAngles(2, 1, 0);
        //ZYX ,yaw is ea(0)
        return ea(0);
    }
} // namespace cane_planner
