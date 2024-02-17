#ifndef RCA_INTERFACE_H
#define RCA_INTERFACE_H
#include <rclcpp/rclcpp.hpp>
#include <hardware_interface/system_interface.hpp>
#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <vector>
#include <libserial/SerialPort.h>
#include <string>
#include "rclcpp_lifecycle/lifecycle_node.hpp"


namespace rca_controller
{

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class RcaInterface : public hardware_interface::SystemInterface , rclcpp_lifecycle::LifecycleNode
{
public:
    RcaInterface();
    virtual ~RcaInterface();


    virtual CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
    virtual CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

    virtual CallbackReturn on_init(const hardware_interface::HardwareInfo & hardware_info) override;
    virtual std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
    virtual std::vector<hardware_interface::CommandInterface>export_command_interfaces() override;

    virtual hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period)  override;
    virtual hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
    LibSerial::SerialPort arduino_;
    std::string port_;
    rclcpp_lifecycle::LifecycleNode base_publisher_;

    std::vector<double> position_commands_;
    std::vector<double> prev_position_commands_;
    std::vector<double> position_states_;


};
}
#endif