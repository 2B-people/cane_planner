#include <plan_manager.h>
#include <path_searching/astar.h>

using namespace cane_planner;

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "cane_planner_node");
    ros::NodeHandle nh("~");

    int planner;
    bool simulation;
    nh.param("planner_node/planner", planner, 1);
    nh.param("planner_node/planner", simulation, true);

    PlannerManager plan_manage(planner,simulation);
    plan_manage.init(nh);
    ros::Duration(1.0).sleep();
    ros::spin();
    return 0;
}
