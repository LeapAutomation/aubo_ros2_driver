#include "aubo_hardware_interface.h"
#include <pluginlib/class_list_macros.hpp>
#include "rclcpp/rclcpp.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include <ctime>
namespace aubo_driver {

AuboHardwareInterface::~AuboHardwareInterface()
{
    stopServoMode();
    enableRobot(false, 5.0);
}
bool AuboHardwareInterface::OnActive()
{
    const std::string robot_ip_ = info_.hardware_parameters["robot_ip"];
    rpc_client_ = std::make_shared<RpcClient>();

    rpc_client_->setRequestTimeout(1000);
    // 接口调用: 连接到 RPC 服务
    rpc_client_->connect(robot_ip_, 30004);
    // 接口调用: 登录
    rpc_client_->login("aubo", "123456");

    rtde_client_ = std::make_shared<RtdeClient>();
    // 接口调用: 连接到 RTDE 服务
    rtde_client_->connect(robot_ip_, 30010);
    // 接口调用: 登录
    rtde_client_->login("aubo", "123456");
    int topic = rtde_client_->setTopic(false, { "R1_message" }, 200, 0);
    if (topic < 0) {
        std::cout << "Set topic fail!" << std::endl;
    }
    rtde_client_->subscribe(topic, [](InputParser &parser) {
        arcs::common_interface::RobotMsgVector msgs;
        msgs = parser.popRobotMsgVector();
        for (size_t i = 0; i < msgs.size(); i++) {
            auto &msg = msgs[i];
        }
    });
    robot_name_ = rpc_client_->getRobotNames().front();

    rpc_client_->getRobotInterface(robot_name_)
    ->getRobotConfig()
    ->setHardwareCustomParameters("[joint_func] \n vff_enable = false\n");

    std::cout << "vff_enable = false" << std::endl;

    // 设置rtde输入
    setInput(rtde_client_);

    // 配置输出
    configSubscribe(rtde_client_);

    enableRobot(true, 5.0);

    startServoMode();

    return true;
}

hardware_interface::CallbackReturn AuboHardwareInterface::on_init(
    const hardware_interface::HardwareInfo &system_info)
{
    if (hardware_interface::SystemInterface::on_init(system_info) !=
        hardware_interface::CallbackReturn::SUCCESS) {
        return hardware_interface::CallbackReturn::ERROR;
    }

    info_ = system_info;
    initialized_ = false;

    for (const hardware_interface::ComponentInfo &joint : info_.joints) {
        // RRBotSystemPositionOnly has exactly one state and command interface
        // on each joint
        if (joint.command_interfaces.size() != 2) {
            RCLCPP_FATAL(
                rclcpp::get_logger("RRBotSystemPositionOnlyHardware"),
                "Joint '%s' has %zu command interfaces found. 1 expected.",
                joint.name.c_str(), joint.command_interfaces.size());
            return hardware_interface::CallbackReturn::ERROR;
        }

        if (joint.command_interfaces[0].name !=
            hardware_interface::HW_IF_POSITION) {
            RCLCPP_FATAL(
                rclcpp::get_logger("RRBotSystemPositionOnlyHardware"),
                "Joint '%s' have %s command interfaces found. '%s' expected.",
                joint.name.c_str(), joint.command_interfaces[0].name.c_str(),
                hardware_interface::HW_IF_POSITION);
            return hardware_interface::CallbackReturn::ERROR;
        }

        if (joint.state_interfaces.size() != 2) {
            RCLCPP_FATAL(rclcpp::get_logger("RRBotSystemPositionOnlyHardware"),
                         "Joint '%s' has %zu state interface. 1 expected.",
                         joint.name.c_str(), joint.state_interfaces.size());
            return hardware_interface::CallbackReturn::ERROR;
        }

        if (joint.state_interfaces[0].name !=
            hardware_interface::HW_IF_POSITION) {
            RCLCPP_FATAL(rclcpp::get_logger("RRBotSystemPositionOnlyHardware"),
                         "Joint '%s' have %s state interface. '%s' expected.",
                         joint.name.c_str(),
                         joint.state_interfaces[0].name.c_str(),
                         hardware_interface::HW_IF_POSITION);
            return hardware_interface::CallbackReturn::ERROR;
        }
    }

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn AuboHardwareInterface::on_shutdown(
        const rclcpp_lifecycle::State &previous_state)
{
    RCLCPP_INFO(rclcpp::get_logger("AuboHardwareInterface"),
                "Shutting down ...please wait...");
    stopServoMode();
    enableRobot(false, 5.0);
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn AuboHardwareInterface::on_error(
        const rclcpp_lifecycle::State &previous_state)
{
    RCLCPP_INFO(rclcpp::get_logger("AuboHardwareInterface"),
                "Error occurred ...please wait...");
    if (!resetErrors(5.0))
    {
        RCLCPP_ERROR(rclcpp::get_logger("AuboHardwareInterface"),
                     "Failed to reset errors on the robot.");
        return hardware_interface::CallbackReturn::ERROR;
    }
    if (!enableRobot(true, 5.0))
    {
        RCLCPP_ERROR(rclcpp::get_logger("AuboHardwareInterface"),
                     "Failed to enable the robot after error reset.");
        return hardware_interface::CallbackReturn::ERROR;
    }
    if (startServoMode() != 0) {
        RCLCPP_ERROR(rclcpp::get_logger("AuboHardwareInterface"),
                     "Failed to start servo mode after error reset.");
        return hardware_interface::CallbackReturn::ERROR;
    }
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn AuboHardwareInterface::on_activate(
    const rclcpp_lifecycle::State &previous_state)
{
    RCLCPP_INFO(rclcpp::get_logger("AuboHardwareInterface"),
                "Starting ...please wait...");
    OnActive();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    readActualQ();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    if (!initialized_) {
        //获取初始状态
        aubo_position_commands_ = actual_q_copy_;
        initialized_ = true;
    }
    return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
AuboHardwareInterface::export_state_interfaces()
{
    std::vector<hardware_interface::StateInterface> state_interfaces;
    for (std::size_t i = 0; i < info_.joints.size(); ++i) {
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.joints[i].name, hardware_interface::HW_IF_POSITION,
            &actual_q_copy_[i]));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.joints[i].name, hardware_interface::HW_IF_VELOCITY,
            &joint_velocity_copy_[i]));
    }

    return state_interfaces;
}
std::vector<hardware_interface::CommandInterface>
AuboHardwareInterface::export_command_interfaces()
{
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    for (std::size_t i = 0; i < info_.joints.size(); ++i) {
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            info_.joints[i].name, hardware_interface::HW_IF_POSITION,
            &aubo_position_commands_[i]));
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            info_.joints[i].name, hardware_interface::HW_IF_VELOCITY,
            &aubo_velocity_commands_[i]));
    }

    return command_interfaces;
}

hardware_interface::return_type AuboHardwareInterface::read(
    const rclcpp::Time &time, const rclcpp::Duration &period)
{
    readActualQ();
    if (!initialized_) {
        //获取初始状态
        aubo_position_commands_ = actual_q_copy_;
        initialized_ = true;
    }

    return hardware_interface::return_type::OK;
}
hardware_interface::return_type AuboHardwareInterface::write(
    const rclcpp::Time &time, const rclcpp::Duration &period)
{
    if (robot_mode_ == RobotModeType::Running && (safety_mode_ == 
        SafetyModeType::Normal || safety_mode_ == SafetyModeType::ReducedMode)) {
        try {
            Servoj(aubo_position_commands_);
        } catch (const std::exception &e) {
        }
    }else{
        // 机器人状态异常
        RCLCPP_WARN_STREAM(
            rclcpp::get_logger("AuboHardwareInterface"),
            "Robot not in valid state for motion command. Plz check&fix robot status firstly then restart driver"
            << "robot_mode_: " << static_cast<int>(robot_mode_)
            << ", safety_mode_: " << static_cast<int>(safety_mode_));

        return hardware_interface::return_type::ERROR;
    }

    return hardware_interface::return_type::OK;
}

void AuboHardwareInterface::readActualQ()
{
    // 使用 actual_q_copy_
    // 固定该时间戳下read到的位姿，否则读取到的关节状态不稳定
    // actual_q_copy_必须用 array 否则会 bad_alloc
    {
        std::unique_lock<std::mutex> lck(rtde_mtx_);
        actual_q_copy_[0] = actual_q_[0];
        actual_q_copy_[1] = actual_q_[1];
        actual_q_copy_[2] = actual_q_[2];
        actual_q_copy_[3] = actual_q_[3];
        actual_q_copy_[4] = actual_q_[4];
        actual_q_copy_[5] = actual_q_[5];

    //获取机械臂关节速度

        joint_velocity_copy_[0] = joint_velocity_[0];
        joint_velocity_copy_[1] = joint_velocity_[1];
        joint_velocity_copy_[2] = joint_velocity_[2];
        joint_velocity_copy_[3] = joint_velocity_[3];
        joint_velocity_copy_[4] = joint_velocity_[4];
        joint_velocity_copy_[5] = joint_velocity_[5];
    }
}
// 设置rtde输入

bool AuboHardwareInterface::enableRobot(bool enable, double time_out_s)
{
    if (enable)
    {
        if (robot_mode_ == RobotModeType::PowerOff)
        {
            if (rpc_client_->getRobotInterface(robot_name_)->getRobotManage()->poweron() != 0)
            {
                RCLCPP_ERROR(rclcpp::get_logger("AuboHardwareInterface"), "Could not power on the robot");
                return false;
            }
            // wait for robot to change mode
            if (!waitForRobotModeChangeFrom(RobotModeType::PowerOff, time_out_s))
            {
                return false;
            }

        }
        if (robot_mode_ == RobotModeType::PowerOn)
        {
            RCLCPP_INFO(rclcpp::get_logger("AuboHardwareInterface"), "Waiting for robot to read initial state");
            auto start_time = std::chrono::steady_clock::now();
            if (!waitForRobotModeChangeTo(RobotModeType::Idle, time_out_s))
            {
                return false;
            }
        }
        if (robot_mode_ == RobotModeType::Idle)
        {
            if (rpc_client_->getRobotInterface(robot_name_)->getRobotManage()->startup() != 0)
            {
                RCLCPP_ERROR(rclcpp::get_logger("AuboHardwareInterface"), "Could not startup the robot");
                return false;
            }
            // wait for robot to change mode
            if (!waitForRobotModeChangeFrom(RobotModeType::Idle, time_out_s))
            {
                return false;
            }
        }
        if (robot_mode_ == RobotModeType::BrakeReleasing)
        {
            RCLCPP_INFO(rclcpp::get_logger("AuboHardwareInterface"), "Waiting for robot to start up");
            if (!waitForRobotModeChangeTo(RobotModeType::Running, time_out_s))
            {
                return false;
            }
        }

        return robot_mode_ == RobotModeType::Running;
    }
    else
    {
        if (robot_mode_ == RobotModeType::PowerOn || robot_mode_ == RobotModeType::Idle
            || robot_mode_ == RobotModeType::Running)
        {
            auto current_mode = robot_mode_;
            RCLCPP_INFO(rclcpp::get_logger("AuboHardwareInterface"), "Disabling robot...");
            if (rpc_client_->getRobotInterface(robot_name_)->getRobotManage()->	poweroff() != 0)
            {
                RCLCPP_ERROR(rclcpp::get_logger("AuboHardwareInterface"), "Could not shutdown the robot");
                return false;
            }
            // wait for robot to change mode
            if (!waitForRobotModeChangeFrom(current_mode, time_out_s))
            {
                return false;
            }
        }
        if (robot_mode_ == RobotModeType::PowerOffing)
        {
            RCLCPP_INFO(rclcpp::get_logger("AuboHardwareInterface"), "Waiting for robot to shutdown");
            auto start_time = std::chrono::steady_clock::now();
            if (!waitForRobotModeChangeTo(RobotModeType::PowerOff, time_out_s))
            {
                return false;
            }
        }

        return robot_mode_ == RobotModeType::PowerOff;
    }
}

bool AuboHardwareInterface::resetErrors(double time_out_s)
{
    RCLCPP_INFO(rclcpp::get_logger("AuboHardwareInterface"), "Resetting robot errors...");
    if (safety_mode_ != SafetyModeType::Normal && safety_mode_ != SafetyModeType::ReducedMode)
    {
        auto current_mode = safety_mode_;
        if (rpc_client_->getRobotInterface(robot_name_)->getRobotManage()->setUnlockProtectiveStop() != 0)
        {
            RCLCPP_ERROR(rclcpp::get_logger("AuboHardwareInterface"), "Could not reset robot protective stop");
            return false;
        }
        // wait for robot to change mode
        auto start_time = std::chrono::steady_clock::now();
        while (safety_mode_ != current_mode)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            if (std::chrono::steady_clock::now() - start_time > std::chrono::duration<double>(time_out_s))
            {
                RCLCPP_ERROR(rclcpp::get_logger("AuboHardwareInterface"), "Timeout waiting for robot to reset protective stop");
                return false;
            }
        }
    }

    return safety_mode_ == SafetyModeType::Normal || safety_mode_ == SafetyModeType::ReducedMode;
}

bool AuboHardwareInterface::isServoModeStart()
{
    return servo_mode_start_;
}
int AuboHardwareInterface::startServoMode()
{
    if (servo_mode_start_) {
        return 0;
    }
    // 接口调用 : 获取机器人的名字
    auto robot_name = rpc_client_->getRobotNames().front();

    //开启servo模式
    rpc_client_->getRobotInterface(robot_name)
        ->getMotionControl()
        ->setServoModeSelect(2);
    int i = 0;
    while (rpc_client_->getRobotInterface(robot_name)
                ->getMotionControl()
                ->getServoModeSelect() != 2) {
        if (i++ > 5) {
            std::cout << "Servo Mode enable fail! Servo Mode is "
                      << rpc_client_->getRobotInterface(robot_name)
                             ->getMotionControl()
                             ->getServoModeSelect()
                      << std::endl;
            return -1;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    servo_mode_start_ = true;
    return 0;
}

int AuboHardwareInterface::stopServoMode()
{
    if (!servo_mode_start_) {
        return 0;
    }
    // 接口调用 : 获取机器人的名字
    auto robot_name = rpc_client_->getRobotNames().front();

    while (!rpc_client_->getRobotInterface(robot_name)
                ->getRobotState()
                ->isSteady()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    // 关闭servo模式
    int i = 0;
    rpc_client_->getRobotInterface(robot_name)
        ->getMotionControl()
        ->setServoModeSelect(0);
    while (rpc_client_->getRobotInterface(robot_name)
               ->getMotionControl()
               ->getServoModeSelect() != 0) {
        if (i++ > 5) {
            std::cout << "Servo Mode disable fail! Servo Mode is "
                      << rpc_client_->getRobotInterface(robot_name)
                             ->getMotionControl()
                             ->getServoModeSelect()
                      << std::endl;
            return -1;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    std::cout << "Servoj end" << std::endl;
    servo_mode_start_ = false;
    return 0;
}

int AuboHardwareInterface::Servoj(
    const std::array<double, 6> joint_position_command)
{
    // 接口调用 : 获取机器人的名字
    auto robot_name = rpc_client_->getRobotNames().front();

    std::vector<double> traj(6, 0);
    for (size_t i = 0; i < traj.size(); i++) {
        traj[i] = joint_position_command[i];
    }
    
    if(rpc_client_->getRobotInterface(robot_name)
                ->getMotionControl()
                ->getServoModeSelect() != 2){
                
        rpc_client_->getRobotInterface(robot_name)
        ->getMotionControl()
        ->setServoModeSelect(2);
    }
    // 接口调用: servoJoint
    while (true) {
        int servoJoint_num = rpc_client_->getRobotInterface(robot_name)
                                ->getMotionControl()
                                ->servoJoint(traj, 0.2, 0.2, 0.01, 0.1, 200);
        if(servoJoint_num != 2){
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }

    return 0;
}
// 设置rtde输入
void AuboHardwareInterface::setInput(RtdeClientPtr cli)
{
    // 接口调用: 发布
    // 组合设置输入
    int topic5 = cli->setTopic(
        true,
        { "input_bit_registers0_to_31", "input_bit_registers32_to_63",
          "input_bit_registers64_to_127", "input_int_registers_0" },
        1, 5);

    std::vector<int> value = { 0x00ff, 0x00, 0x00, 44 };
    cli->publish(
        5, [value](arcs::aubo_sdk::OutputBuilder &ro) { ro.push(value); });

    int topic6 = cli->setTopic(
        true, { "input_float_registers_0", "input_double_registers_1" }, 1, 6);

    std::vector<double> value2 = { 3.1, 4.1 };
    cli->publish(
        6, [value2](arcs::aubo_sdk::OutputBuilder &ro) { ro.push(value2); });
}
void AuboHardwareInterface::configSubscribe(RtdeClientPtr cli)
{
    // 接口调用: 设置 topic1
    int topic1 = cli->setTopic(
        false,
        { "R1_actual_q", "R1_actual_qd", "R1_robot_mode", "R1_safety_mode",
          "runtime_state", "line_number", "R1_actual_TCP_pose" },
        500, 0);
    // 接口调用: 订阅
    cli->subscribe(topic1, [this](InputParser &parser) {
        std::unique_lock<std::mutex> lck(rtde_mtx_);
        actual_q_ = parser.popVectorDouble();
        joint_velocity_ = parser.popVectorDouble();
        robot_mode_ = parser.popRobotModeType();
        safety_mode_ = parser.popSafetyModeType();
        runtime_state_ = parser.popRuntimeState();
        line_ = parser.popInt32();
        actual_TCP_pose_ = parser.popVectorDouble();
    });
}

bool AuboHardwareInterface::waitForRobotModeChangeTo(RobotModeType target_mode, double time_out_s)
{
    auto start_time = std::chrono::steady_clock::now();
    while (robot_mode_ != target_mode)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        if (std::chrono::steady_clock::now() - start_time > std::chrono::duration<double>(time_out_s))
        {
            RCLCPP_ERROR(rclcpp::get_logger("AuboHardwareInterface"), "Timeout waiting for robot to change mode");
            return false;
        }
    }
    return true;
}

bool AuboHardwareInterface::waitForRobotModeChangeFrom(RobotModeType source_mode, double time_out_s)
{
    auto start_time = std::chrono::steady_clock::now();
    while (robot_mode_ == source_mode)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        if (std::chrono::steady_clock::now() - start_time > std::chrono::duration<double>(time_out_s))
        {
            RCLCPP_ERROR(rclcpp::get_logger("AuboHardwareInterface"), "Timeout waiting for robot to change mode");
            return false;
        }
    }
    return true;
}
} // namespace aubo_driver

PLUGINLIB_EXPORT_CLASS(aubo_driver::AuboHardwareInterface,
                       hardware_interface::SystemInterface)
