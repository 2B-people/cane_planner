#include <gkf_control/gkf_hardware_interface.h>

GKFHardwareInterface::GKFHardwareInterface(ros::NodeHandle &nh) : nh_(nh)
{
    init();
    controller_manager_.reset(new controller_manager::ControllerManager(this, nh_));
    loop_hz_ = 50;
    ros::Duration update_freq = ros::Duration(1.0 / loop_hz_);

    pub = nh_.advertise<rospy_tutorials::Floats>("/joints_to_aurdino", 10);
    // client = nh_.serviceClient<three_dof_planar_manipulator::Floats_array>("/read_joint_state");

    non_realtime_loop_ = nh_.createTimer(update_freq, &GKFHardwareInterface::update, this);
}

GKFHardwareInterface::~GKFHardwareInterface()
{
    ser_write("zS\n");
    ROS_INFO_STREAM("stop connect");
    ser_.close();
}

void GKFHardwareInterface::init()
{

    joint_name_ = "joint1";

    // Create joint state interface
    hardware_interface::JointStateHandle jointStateHandle(joint_name_, &joint_position_, &joint_velocity_, &joint_effort_);
    joint_state_interface_.registerHandle(jointStateHandle);

    // Create position joint interface
    // hardware_interface::JointHandle jointPositionHandle(jointStateHandle, &joint_position_command_);
    // position_joint_interface_.registerHandle(jointPositionHandle);

    // Create velocity joint interface
    // hardware_interface::JointHandle jointVelocityHandle(jointStateHandle, &joint_velocity_command_);
    // effort_joint_interface_.registerHandle(jointVelocityHandle);

    // Create effort joint interface
    hardware_interface::JointHandle jointEffortHandle(jointStateHandle, &joint_effort_command_);
    effort_joint_interface_.registerHandle(jointEffortHandle);

    // Create Joint Limit interface
    joint_limits_interface::JointLimits limits;
    joint_limits_interface::getJointLimits("joint1", nh_, limits);
    joint_limits_interface::EffortJointSaturationHandle jointLimitsHandle(jointEffortHandle, limits);
    effortJointSaturationInterface.registerHandle(jointLimitsHandle);

    // Register all joints interfaces
    registerInterface(&joint_state_interface_);
    registerInterface(&position_joint_interface_);
    registerInterface(&effort_joint_interface_);
    registerInterface(&effortJointSaturationInterface);

    // Serial
    std::string port("/dev/feedback_controller");
    int baudrate = 115200;

    nh_.param("port", port, port);
    nh_.param("baudrate", baudrate, baudrate);
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
    ser_write("zO\n");
}

void GKFHardwareInterface::update(const ros::TimerEvent &e)
{
    elapsed_time_ = ros::Duration(e.current_real - e.last_real);
    read(elapsed_time_);
    controller_manager_->update(ros::Time::now(), elapsed_time_);
    write(elapsed_time_);
}

void GKFHardwareInterface::read(ros::Duration elapsed_time)
{
    u_char recv_data[200];
    int encoder = 0;
    auto time = elapsed_time.toSec();
    if (ser_.available() != 0)
    {
        auto size_data = ser_.read(recv_data, ser_.available());
        std::string recv_string(reinterpret_cast<char *>(&recv_data[0]), size_data);
        // ROS_INFO("%lf: reading %s", time, recv_string.c_str());
        auto s_index = recv_string.find("E");
        auto q_index = recv_string.find("\n");
        if (s_index != std::string::npos && q_index != std::string::npos)
        {
            std::string number_str = recv_string.substr(s_index + 1, q_index - s_index - 1);
            std::istringstream(number_str) >> encoder;
        }
        ROS_INFO("encoder is %d", encoder);
    }

    joint_position_ = 0;
    joint_velocity_ = 0;
}

void GKFHardwareInterface::write(ros::Duration elapsed_time)
{

    effortJointSaturationInterface.enforceLimits(elapsed_time);
    joints_pub.data.clear();
    joints_pub.data.push_back(joint_effort_command_);

    // TEST
    joint_effort_command_ = 2000;

    ROS_INFO("PWM Cmd: %.2f", joint_effort_command_);
    std::string send_data = "z" + std::to_string((int)(joint_effort_command_)) + "\n";
    ser_write(send_data);
    pub.publish(joints_pub);
}

// 串口辅助函数
void GKFHardwareInterface::ser_write(std::string send_data)
{
    u_char send_data_char[send_data.size()];
    for (size_t i = 0; i < send_data.size(); i++)
        send_data_char[i] = send_data.c_str()[i];
    ser_.write(send_data_char, send_data.size());
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "single_joint_hardware_interface");
    ros::NodeHandle nh;
    // ros::AsyncSpinner spinner(4);
    ros::MultiThreadedSpinner spinner(2); // Multiple threads for controller service callback and for the Service client callback used to get the feedback from ardiuno
    GKFHardwareInterface GKF(nh);
    // spinner.start();
    spinner.spin();
    // ros::spin();
    return 0;
}
