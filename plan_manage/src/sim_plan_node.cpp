#include <plan_manager.h>
#include <path_searching/astar.h>

using namespace cane_planner;

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "sim_plan");
    ros::NodeHandle nh("~");

    int planner;
    bool simulation;
    nh.param("planner_node/planner", planner, 1);
    nh.param("planner_node/simulation", simulation, true);

    PlannerManager plan_manage(simulation);
    plan_manage.simInit(nh);
    ros::Duration(1.0).sleep();
    while (ros::ok())
    {
        ros::Duration(0.1).sleep();
        plan_manage.callPath();
        ros::spinOnce();
    }

    return 0;
}
