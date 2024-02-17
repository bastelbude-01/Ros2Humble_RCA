#include "rca_controller/rca_interface.hpp"
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>
#include "std_msgs/msg/float32.hpp"


namespace rca_controller
{
RcaInterface::RcaInterface()
{

}

RcaInterface::~RcaInterface()
{
    if(arduino_.IsOpen())
    {
        try
        {
            arduino_.Close();
        }
        catch(...)
        {
            RCLCPP_FATAL_STREAM(rclcpp::get_logger("RcaInterface"),
            "Etwas ist schief gelaufen beim Schliessen der Serialen Verbindung -> Störung Port :" << port_);

        }
        
    }
}

CallbackReturn RcaInterface::on_init(const hardware_interface::HardwareInfo & hardware_info)
{
    CallbackReturn result = hardware_interface::SystemInterface::on_init(hardware_info);

    if(result != CallbackReturn::SUCCESS)
    {
        return result;
    }
    try
    {
        port_ = info_.hardware_parameters.at("port");
    }
    catch(const std::out_of_range &e)
    {
        RCLCPP_FATAL(rclcpp::get_logger("RcaInterface"), "Kein Serial Port gefunden, Abbruch");
        return CallbackReturn::FAILURE;
    }

    position_commands_.reserve(info_.joints.size());
    position_states_.reserve(info_.joints.size());
    prev_position_commands_.reserve(info_.joints.size());


    return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> RcaInterface::export_state_interfaces() 
{
    std::vector<hardware_interface::StateInterface> state_interfaces;
    for(size_t i=0; i<info_.joints.size(); i++)
    {
        state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &position_states_[i]));
    }
    return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> RcaInterface::export_command_interfaces()
{
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    for(size_t i=0; i<info_.joints.size(); i++)
    {
        command_interfaces.emplace_back(hardware_interface::CommandInterface(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &position_commands_[i]));
    }
    return command_interfaces;
}


CallbackReturn RcaInterface::on_activate(const rclcpp_lifecycle::State & previous_state) 
{
    RCLCPP_INFO(rclcpp::get_logger("RcaInterface"),"Arm Hardware wird gestartet... Bitte Warten...");
    position_commands_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    prev_position_commands_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    position_states_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    try
    {
        arduino_.Open(port_);
        arduino_.SetBaudRate(LibSerial::BaudRate::BAUD_115200);        
    }
    catch(...)
    {
        RCLCPP_FATAL_STREAM(rclcpp::get_logger("RcaInterface"),"Verbinung zum USB-Port Fehlgeschlagen! Check : " << port_);
        return CallbackReturn::FAILURE;
    }
    RCLCPP_INFO(rclcpp::get_logger("RcaInterface"),"Hardware erfolgreich gestartet... Erwarte Befehle.");
    return CallbackReturn::SUCCESS;

}


CallbackReturn RcaInterface::on_deactivate(const rclcpp_lifecycle::State & previous_state) 
{
    RCLCPP_INFO(rclcpp::get_logger("RcaInterface"),"Hardware wird herunter gefahren... Auf Wiedersehen...");
    if(arduino_.IsOpen())
    {
        try
        {
            arduino_.Close();
        }
        catch(...)
        {
            RCLCPP_FATAL_STREAM(rclcpp::get_logger("RcaInterface"),"Schliessen des USB-Port Fehlgeschlagen! Check : " << port_);
            return CallbackReturn::FAILURE;
        }
    }
    RCLCPP_INFO(rclcpp::get_logger("RcaInterface"),"Hardware erfolgreich ausgeschaltet.");
    return CallbackReturn::SUCCESS;
}

hardware_interface::return_type RcaInterface::read(const rclcpp::Time & time, const rclcpp::Duration & period) 
{
    position_states_ = position_commands_;
    return hardware_interface::return_type::OK;
}



hardware_interface::return_type RcaInterface::write(const rclcpp::Time & time, const rclcpp::Duration & period)
{
    if(position_commands_ == prev_position_commands_)
    {
        return hardware_interface::return_type::OK;
    }

    std::string msg;
    int base = static_cast<int>(((position_commands_.at(0) + (M_PI/2))*180)/M_PI); // base
    msg.append("b");
    msg.append(std::to_string(base));
    msg.append(",");

    int shoulder = 180 - static_cast<int>(((position_commands_.at(1)+(M_PI/2))*180)/M_PI); // arm1 lang
    msg.append("s");
    msg.append(std::to_string(shoulder));
    msg.append(",");

    int arm = 180 - static_cast<int>(((position_commands_.at(2)+(M_PI/2))*180)/M_PI); // arm2 kurz
    msg.append("a");
    msg.append(std::to_string(arm));
    msg.append(",");

    int rotate = static_cast<int>(((position_commands_.at(3) + (M_PI/2))*180)/M_PI);  // arm3 rotate
    msg.append("r");
    msg.append(std::to_string(rotate));
    msg.append(",");

    int elbow = static_cast<int>(((position_commands_.at(4) + (M_PI/2))+180)/M_PI);  // nick
    msg.append("e");
    msg.append(std::to_string(elbow));
    msg.append(",");    

    int gripper = static_cast<int>((-position_commands_.at(5)*180)/(M_PI/2));  // gripper
    msg.append("g");
    msg.append(std::to_string(gripper));
    msg.append(",");

    RCLCPP_WARN_STREAM(rclcpp::get_logger("RcaInterface"),"Versendete Nachricht -> " << msg);

    try
    {
        arduino_.Write(msg);
    }
    catch(...)
    {
        RCLCPP_ERROR_STREAM(rclcpp::get_logger("RcaInterface"),"Versenden der Nachricht Fehlgeschlagen -> " << msg);
        return hardware_interface::return_type::ERROR;
    }
    prev_position_commands_ = position_commands_;
    return hardware_interface::return_type::OK;
}

}

PLUGINLIB_EXPORT_CLASS(rca_controller::RcaInterface, hardware_interface::SystemInterface);