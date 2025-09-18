
#include "aubo_hardware_interface.h"

#include <algorithm>
#include <pluginlib/class_list_macros.hpp>

namespace aubo_driver
{

AuboHardwareInterface::~AuboHardwareInterface()
{
    // Ensure motion stops and transport is closed
    (void)stopServoMode();
    disconnectClients();
}

hardware_interface::CallbackReturn AuboHardwareInterface::on_init(const hardware_interface::HardwareInfo &info)
{
    if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
    {
        return hardware_interface::CallbackReturn::ERROR;
    }

    // Parameters from URDF <ros2_control> hardware parameters
    try
    {
        robot_ip_ = info.hardware_parameters.at("robot_ip");
        if (info.hardware_parameters.count("username"))
        {
            username_ = info.hardware_parameters.at("username");
        }
        if (info.hardware_parameters.count("password"))
        {
            password_ = info.hardware_parameters.at("password");
        }
        if (info.hardware_parameters.count("auto_power_on"))
        {
            auto_power_on_ = info.hardware_parameters.at("auto_power_on") == "true" ||
                             info.hardware_parameters.at("auto_power_on") == "True";
        }
        if (info.hardware_parameters.count("power_off_on_disable_"))
        {
            power_off_on_disable_ = info.hardware_parameters.at("power_off_on_disable_") == "true" ||
                                    info.hardware_parameters.at("power_off_on_disable_") == "True";
        }
        if (info.hardware_parameters.count("start_servo_on_activate"))
        {
            start_servo_on_activate_ = info.hardware_parameters.at("start_servo_on_activate") == "true" ||
                                       info.hardware_parameters.at("start_servo_on_activate") == "True";
        }
        if (info.hardware_parameters.count("sim_mode"))
        {
            sim_mode_ =
                info.hardware_parameters.at("sim_mode") == "true" || info.hardware_parameters.at("sim_mode") == "True";
        }
        if (info.hardware_parameters.count("rtde_frequency_hz"))
        {
            rtde_frequency_hz_ = std::stoi(info.hardware_parameters.at("rtde_frequency_hz"));
        }

        RCLCPP_INFO(get_logger(), "Hardware parameters:");
        RCLCPP_INFO(get_logger(), "  robot_ip: %s", robot_ip_.c_str());
        RCLCPP_INFO(get_logger(), "  username: %s", username_.c_str());
        RCLCPP_INFO(get_logger(), "  password: %s", password_.c_str());
        RCLCPP_INFO(get_logger(), "  auto_power_on: %s", auto_power_on_ ? "true" : "false");
        RCLCPP_INFO(get_logger(), "  power_off_on_disable: %s", power_off_on_disable_ ? "true" : "false");
        RCLCPP_INFO(get_logger(), "  start_servo_on_activate: %s", start_servo_on_activate_ ? "true" : "false");
        RCLCPP_INFO(get_logger(), "  aubo sim_mode: %s", sim_mode_ ? "true" : "false");
    }
    catch (const std::exception &e)
    {
        RCLCPP_FATAL(get_logger(), "Missing/invalid hardware parameters: %s", e.what());
        return hardware_interface::CallbackReturn::ERROR;
    }

    // Validate joint interfaces
    for (const auto &joint : info_.joints)
    {
        if (joint.command_interfaces.size() != 2 ||
            joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION ||
            joint.command_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
        {
            RCLCPP_FATAL(
                get_logger(), "Joint '%s' must expose position & velocity command interfaces", joint.name.c_str()
            );
            return hardware_interface::CallbackReturn::ERROR;
        }
        if (joint.state_interfaces.size() < 1 || joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
        {
            RCLCPP_FATAL(get_logger(), "Joint '%s' must expose position state interface", joint.name.c_str());
            return hardware_interface::CallbackReturn::ERROR;
        }
    }

    initialized_ = false;
    RCLCPP_INFO(get_logger(), "on_init complete for IP %s", robot_ip_.c_str());
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn AuboHardwareInterface::on_activate(const rclcpp_lifecycle::State &)
{
    RCLCPP_INFO(get_logger(), "Activating hardware interface...");

    if (!connectClients())
    {
        RCLCPP_ERROR(get_logger(), "Failed to connect to RPC/RTDE servers");
        return hardware_interface::CallbackReturn::ERROR;
    }

    if (!configureRobot())
    {
        RCLCPP_ERROR(get_logger(), "Failed to configure robot");
        return hardware_interface::CallbackReturn::ERROR;
    }

    // Advertise enable/disable service while active
    auto node = get_node();
    enable_robot_srv_ = node->create_service<std_srvs::srv::SetBool>(
        "aubo/enable_robot",
        std::bind(&AuboHardwareInterface::enableRobotCb, this, std::placeholders::_1, std::placeholders::_2)
    );

    // Optionally power on and start servo
    if (auto_power_on_)
    {
        if (!enableRobot(true))
        {
            RCLCPP_ERROR(get_logger(), "Failed to enable robot on activation");
            return hardware_interface::CallbackReturn::ERROR;
        }
        if (start_servo_on_activate_)
        {
            (void)startServoMode();
        }
    }

    // Prime first state snapshot
    copyLatestState();
    cmd_pos_ = q_snapshot_;
    initialized_ = true;
    RCLCPP_INFO(get_logger(), "Activation complete");
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn AuboHardwareInterface::on_deactivate(const rclcpp_lifecycle::State &)
{
    RCLCPP_INFO(get_logger(), "Deactivating hardware interface...");

    (void)stopServoMode();
    (void)powerOffRobot();

    enable_robot_srv_.reset();
    disconnectClients();

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn AuboHardwareInterface::on_shutdown(const rclcpp_lifecycle::State &)
{
    RCLCPP_INFO(get_logger(), "Shutting down hardware interface...");

    (void)stopServoMode();
    (void)powerOffRobot();
    disconnectClients();
    return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> AuboHardwareInterface::export_state_interfaces()
{
    std::vector<hardware_interface::StateInterface> out;
    out.reserve(info_.joints.size() * 2);
    for (size_t i = 0; i < info_.joints.size(); ++i)
    {
        out.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &q_snapshot_[i]);
        out.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &qd_snapshot_[i]);
    }
    return out;
}

std::vector<hardware_interface::CommandInterface> AuboHardwareInterface::export_command_interfaces()
{
    std::vector<hardware_interface::CommandInterface> out;
    out.reserve(info_.joints.size() * 2);
    for (size_t i = 0; i < info_.joints.size(); ++i)
    {
        out.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &cmd_pos_[i]);
        out.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &cmd_vel_[i]);
    }
    return out;
}

hardware_interface::return_type AuboHardwareInterface::read(const rclcpp::Time &, const rclcpp::Duration &)
{
    copyLatestState();
    if (!initialized_)
    {
        cmd_pos_ = q_snapshot_;
        initialized_ = true;
    }
    return hardware_interface::return_type::OK;
}

hardware_interface::return_type AuboHardwareInterface::write(const rclcpp::Time &, const rclcpp::Duration &)
{
    const bool ok_state = (robot_mode_ == RobotModeType::Running) &&
                          (safety_mode_ == SafetyModeType::Normal || safety_mode_ == SafetyModeType::ReducedMode);

    if (!ok_state)
    {
        RCLCPP_WARN_THROTTLE(
            get_logger(), *get_node()->get_clock(), 2000, "Robot not ready for motion. mode=%d safety=%d",
            (int)robot_mode_, (int)safety_mode_
        );
        return hardware_interface::return_type::ERROR;
    }

    if (!servo_mode_enabled_)
    {
        (void)startServoMode();
    }
    (void)servojOnce(cmd_pos_);
    return hardware_interface::return_type::OK;
}

// -------------------------- helpers --------------------------

template <class Pred>
bool AuboHardwareInterface::waitFor(const Pred &pred, std::chrono::milliseconds timeout, std::chrono::milliseconds poll)
    const
{
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline)
    {
        if (pred())
        {
            return true;
        }
        std::this_thread::sleep_for(poll);
    }
    return pred();
}

bool AuboHardwareInterface::connectClients()
{
    try
    {
        rpc_client_ = std::make_shared<RpcClient>();
        rpc_client_->setRequestTimeout(1000);
        rpc_client_->connect(robot_ip_, 30004);
        rpc_client_->login(username_, password_);

        rtde_client_ = std::make_shared<RtdeClient>();
        rtde_client_->connect(robot_ip_, 30010);
        rtde_client_->login(username_, password_);

        // Resolve robot name
        auto names = rpc_client_->getRobotNames();
        if (names.empty())
        {
            RCLCPP_ERROR(get_logger(), "No robots reported by RPC server");
            return false;
        }
        robot_name_ = names.front();

        // RTDE topics
        setInput(rtde_client_);
        configSubscribe(rtde_client_);
        return true;
    }
    catch (const std::exception &e)
    {
        RCLCPP_ERROR(get_logger(), "Exception during connect: %s", e.what());
        return false;
    }
}

void AuboHardwareInterface::disconnectClients()
{
    // SDK does not provide explicit disconnect in this stub; rely on RAII
    rtde_client_.reset();
    rpc_client_.reset();
}

bool AuboHardwareInterface::configureRobot()
{
    try
    {
        auto iface = rpc_client_->getRobotInterface(robot_name_);

        iface->getRobotConfig()->setHardwareCustomParameters("[joint_func]\n vff_enable = false\n");
        iface->getRobotConfig()->setCollisionLevel(0);
        iface->getRobotManage()->setSim(sim_mode_);
        return true;
    }
    catch (const std::exception &e)
    {
        RCLCPP_ERROR(get_logger(), "configureRobot exception: %s", e.what());
        return false;
    }
}

bool AuboHardwareInterface::enableRobot(bool enable)
{
    try
    {
        auto iface = rpc_client_->getRobotInterface(robot_name_);
        auto manage = iface->getRobotManage();
        auto state = iface->getRobotState();

        if (enable)
        {
            if (state->isPowerOn())
            {
                RCLCPP_DEBUG(get_logger(), "Robot already powered on");
                return true;
            }

            if (!manage->poweron())
            {
                // Wait to IDLE
                bool ok_idle = waitFor([&] { return robot_mode_ == RobotModeType::Idle; }, std::chrono::seconds(60));
                if (!ok_idle)
                {
                    RCLCPP_ERROR(get_logger(), "Timed out waiting for Idle after poweron");
                    return false;
                }
                if (manage->startup() != 0)
                {
                    RCLCPP_ERROR(get_logger(), "Startup command failed");
                    return false;
                }
                // Wait to RUNNING
                bool ok_run = waitFor([&] { return robot_mode_ == RobotModeType::Running; }, std::chrono::seconds(10));
                if (!ok_run)
                {
                    RCLCPP_ERROR(get_logger(), "Timed out waiting for Running after startup");
                    return false;
                }
                return true;
            }
            // If SDK returned true meaning it handled power-on synchronously
            return waitFor([&] { return state->isPowerOn(); }, std::chrono::seconds(5));
        }
        else
        {
            if (!state->isPowerOn())
            {
                return true;
            }
            if (!power_off_on_disable_)
            {
                return stopServoMode() == 0;
            }
            return powerOffRobot() == 0;
        }
    }
    catch (const std::exception &e)
    {
        RCLCPP_ERROR(get_logger(), "enableRobot exception: %s", e.what());
        return false;
    }
}

int AuboHardwareInterface::powerOffRobot()
{
    try
    {
        auto iface = rpc_client_->getRobotInterface(robot_name_);
        auto manage = iface->getRobotManage();
        auto state = iface->getRobotState();

        if (!state->isPowerOn())
        {
            return 0;
        }

        if (!manage->poweroff())
        {
            // Wait for power off
            bool ok = waitFor([&] { return !state->isPowerOn(); }, std::chrono::seconds(10));
            if (!ok)
            {
                RCLCPP_ERROR(get_logger(), "Timed out waiting for power off");
                return -1;
            }
            return 0;
        }
        else
        {
            RCLCPP_ERROR(get_logger(), "Poweroff command failed");
            return -1;
        }
    }
    catch (const std::exception &e)
    {
        RCLCPP_ERROR(get_logger(), "powerOffRobot exception: %s", e.what());
        return -1;
    }
}

int AuboHardwareInterface::startServoMode()
{
    if (servo_mode_enabled_)
    {
        return 0;
    }
    try
    {
        auto mc = rpc_client_->getRobotInterface(robot_name_)->getMotionControl();
        mc->setServoMode(true);
        bool ok = waitFor([&] { return mc->isServoModeEnabled(); }, std::chrono::milliseconds(200));
        servo_mode_enabled_ = ok;
        if (!ok)
        {
            RCLCPP_ERROR(get_logger(), "Failed to enable servo mode");
            return -1;
        }
        return 0;
    }
    catch (...)
    {
        RCLCPP_ERROR(get_logger(), "Exception enabling servo mode");
        return -1;
    }
}

int AuboHardwareInterface::stopServoMode()
{
    if (!servo_mode_enabled_)
    {
        return 0;
    }
    try
    {
        auto iface = rpc_client_->getRobotInterface(robot_name_);
        auto mc = iface->getMotionControl();

        // wait steady
        (void)waitFor([&] { return iface->getRobotState()->isSteady(); }, std::chrono::milliseconds(300));

        mc->setServoMode(false);
        bool ok = waitFor([&] { return !mc->isServoModeEnabled(); }, std::chrono::milliseconds(200));
        servo_mode_enabled_ = !ok ? servo_mode_enabled_.load() : false;
        if (!ok)
        {
            RCLCPP_WARN(get_logger(), "Timed out disabling servo mode");
            return -1;
        }
        return 0;
    }
    catch (...)
    {
        RCLCPP_ERROR(get_logger(), "Exception disabling servo mode");
        return -1;
    }
}

int AuboHardwareInterface::servojOnce(const std::array<double, 6> &q_cmd)
{
    try
    {
        auto mc = rpc_client_->getRobotInterface(robot_name_)->getMotionControl();
        if (!mc->isServoModeEnabled())
        {
            mc->setServoMode(true);
            (void)waitFor([&] { return mc->isServoModeEnabled(); }, std::chrono::milliseconds(200));
            servo_mode_enabled_ = mc->isServoModeEnabled();
        }

        std::vector<double> traj(q_cmd.begin(), q_cmd.end());

        // Limited, bounded retries if controller returns busy code (2)
        for (int attempt = 0; attempt < 4; ++attempt)
        {
            // according to doc, a and v are not used, t is supposed to be control period but they've set it to 0.01
            int rc = mc->servoJoint(traj, /*a*/ 0.2, /*v*/ 0.2, /*t*/ 0.01, /*lookahead*/ 0.1, /*gain*/ 200);
            if (rc != 2)
            {
                return 0; // anything except queue full is considered success
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
        return 0;
    }
    catch (...)
    {
        RCLCPP_ERROR(get_logger(), "Exception in servojOnce");
        return -1;
    }
}

void AuboHardwareInterface::setInput(RtdeClientPtr cli)
{
    try
    {
        int topic_5 = cli->setTopic(
            true,
            {"input_bit_registers0_to_31", "input_bit_registers32_to_63", "input_bit_registers64_to_127",
             "input_int_registers_0"},
            1, 5
        );
        (void)topic_5;
        std::vector<int> value = {0x00ff, 0x00, 0x00, 44};
        cli->publish(5, [value](arcs::aubo_sdk::OutputBuilder &ro) { ro.push(value); });

        int topic_6 = cli->setTopic(true, {"input_float_registers_0", "input_double_registers_1"}, 1, 6);
        (void)topic_6;
        std::vector<double> value2 = {3.1, 4.1};
        cli->publish(6, [value2](arcs::aubo_sdk::OutputBuilder &ro) { ro.push(value2); });
    }
    catch (...)
    {
        RCLCPP_WARN(get_logger(), "Failed to configure RTDE outputs (non-fatal)");
    }
}

void AuboHardwareInterface::configSubscribe(RtdeClientPtr cli)
{
    try
    {

        /// NOTE: in the original code, there was the following section in the constructor:
        /**
            int topic = rtde_client_->setTopic(false, {"R1_message"}, 500, 0);
            if (topic < 0)
            {
                std::cout << "Set topic fail!" << std::endl;
            }
            rtde_client_->subscribe(
                topic,
                [](InputParser &parser)
                {
                    arcs::common_interface::RobotMsgVector msgs;
                    msgs = parser.popRobotMsgVector();
                    for (size_t i = 0; i < msgs.size(); i++)
                    {
                        auto &msg = msgs[i];
                    }
                }
            );
         */
        /// However, if I understand it correctly, this is just a demo of how to subscribe to a topic because this
        /// channel (0) is then overridden below. So I omitted it here.

        int topic = cli->setTopic(
            false,
            {"R1_actual_q", "R1_actual_qd", "R1_robot_mode", "R1_safety_mode", "runtime_state", "line_number",
             "R1_actual_TCP_pose"},
            rtde_frequency_hz_, 0
        );

        cli->subscribe(
            topic,
            [this](InputParser &parser)
            {
                std::lock_guard<std::mutex> lk(rtde_mtx_);
                actual_q_ = parser.popVectorDouble();
                actual_qd_ = parser.popVectorDouble();
                robot_mode_ = parser.popRobotModeType();
                safety_mode_ = parser.popSafetyModeType();
                runtime_state_ = parser.popRuntimeState();
                line_ = parser.popInt32();
                actual_tcp_ = parser.popVectorDouble();
            }
        );
    }
    catch (...)
    {
        RCLCPP_ERROR(get_logger(), "Failed to configure RTDE subscription");
        throw;
    }
}

void AuboHardwareInterface::copyLatestState()
{
    std::lock_guard<std::mutex> lk(rtde_mtx_);
    for (size_t i = 0; i < 6; ++i)
    {
        q_snapshot_[i] = actual_q_[i];
        qd_snapshot_[i] = actual_qd_[i];
    }
}

void AuboHardwareInterface::enableRobotCb(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> &req,
    const std::shared_ptr<std_srvs::srv::SetBool::Response> &res
)
{
    res->success = enableRobot(req->data);
    res->message = req->data ? (res->success ? "Robot enabled" : "Enable failed")
                             : (res->success ? "Robot disabled" : "Disable failed");
}

} // namespace aubo_driver

PLUGINLIB_EXPORT_CLASS(aubo_driver::AuboHardwareInterface, hardware_interface::SystemInterface)
