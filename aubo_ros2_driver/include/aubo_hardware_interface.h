#ifndef AUBO_HARDWARE_INTERFACE_H
#define AUBO_HARDWARE_INTERFACE_H

#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include <std_srvs/srv/set_bool.hpp>

#include <aubo/robot/robot_state.h>
#include <aubo/type_def.h>

#include "aubo_sdk/rpc.h"
#include "aubo_sdk/rtde.h"
#include "serviceinterface.h"

namespace aubo_driver
{

// Forward decls for AUBO SDK aliases
using arcs::aubo_sdk::InputParser;
using arcs::aubo_sdk::RpcClient;
using arcs::aubo_sdk::RtdeClient;
using arcs::aubo_sdk::RtdeClientPtr;

// Robot enums
using RobotModeType = arcs::aubo_sdk::RobotModeType;
using SafetyModeType = arcs::aubo_sdk::SafetyModeType;
using RuntimeState = arcs::aubo_sdk::RuntimeState;

class AuboHardwareInterface : public hardware_interface::SystemInterface
{
public:
    RCLCPP_SHARED_PTR_DEFINITIONS(AuboHardwareInterface);
    ~AuboHardwareInterface() override;

    // lifecycle hooks
    hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo& info) override;
    hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
    hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;
    hardware_interface::CallbackReturn on_shutdown(const rclcpp_lifecycle::State& previous_state) override;

    // interfaces
    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

    // read & write
    hardware_interface::return_type read(const rclcpp::Time& time, const rclcpp::Duration& period) override;
    hardware_interface::return_type write(const rclcpp::Time& time, const rclcpp::Duration& period) override;

    // services / helpers
    bool isServoModeEnabled() const { return servo_mode_enabled_; }

private:
    // ---- helpers ----
    template <class Pred>
    bool waitFor(
        const Pred& pred, std::chrono::milliseconds timeout,
        std::chrono::milliseconds poll = std::chrono::milliseconds(20)
    ) const;

    bool connectClients();
    void disconnectClients();
    bool configureRobot();
    bool enableRobot(bool enable);
    int startServoMode();
    int stopServoMode();
    int powerOffRobot();
    int servojOnce(const std::array<double, 6>& q_cmd);
    void setInput(RtdeClientPtr cli);
    void configSubscribe(RtdeClientPtr cli);
    void copyLatestState();

    void enableRobotCb(
        const std::shared_ptr<std_srvs::srv::SetBool::Request>& req,
        const std::shared_ptr<std_srvs::srv::SetBool::Response>& res
    );

    // ---- ros / config ----
    std::string robot_ip_;
    std::string username_{"aubo"};
    std::string password_{"123456"};
    bool auto_power_on_{true};
    bool power_off_on_disable_{false};
    bool start_servo_on_activate_{true};
    bool sim_mode_{false};
    int rtde_frequency_hz_{500};

    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr enable_robot_srv_;

    // ---- SDK clients ----
    std::shared_ptr<RpcClient> rpc_client_;
    std::shared_ptr<RtdeClient> rtde_client_;

    // ---- robot identity ----
    std::string robot_name_;

    // ---- joint state/command ----
    std::array<double, 6> cmd_pos_{0, 0, 0, 0, 0, 0};
    std::array<double, 6> cmd_vel_{0, 0, 0, 0, 0, 0};

    std::vector<double> actual_q_ = std::vector<double>(6, 0.0);
    std::vector<double> actual_qd_ = std::vector<double>(6, 0.0);
    std::vector<double> actual_tcp_ = std::vector<double>(6, 0.0);
    std::vector<double> actual_tcp_v_ = std::vector<double>(6, 0.0);

    std::array<double, 6> q_snapshot_{0, 0, 0, 0, 0, 0};
    std::array<double, 6> qd_snapshot_{0, 0, 0, 0, 0, 0};

    // ---- status ----
    std::atomic<bool> initialized_{false};
    std::atomic<bool> servo_mode_enabled_{false};

    RobotModeType robot_mode_ = RobotModeType::NoController;
    SafetyModeType safety_mode_ = SafetyModeType::Normal;
    RuntimeState runtime_state_ = RuntimeState::Stopped;
    int32_t line_{-1};

    // ---- sync ----
    mutable std::mutex rtde_mtx_;
};

} // namespace aubo_driver

#endif // AUBO_HARDWARE_INTERFACE_H
