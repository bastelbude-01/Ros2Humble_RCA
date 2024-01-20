#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <libserial/SerialPort.h>
#include <chrono>


using namespace std::chrono_literals;

class SimpleSerialReceiver : public rclcpp::Node
{
public:
  SimpleSerialReceiver() : Node("simple_serial_receiver")
  {
    declare_parameter<std::string>("port","/dev/ttyUSB0");
    std::string port_ = get_parameter("port").as_string();

    pub_ = create_publisher<std_msgs::msg::String>("serial_receiver", 10);
    timer_ = create_wall_timer(0.01s, std::bind(&SimpleSerialReceiver::timerCallback, this));

    arduino_.Open(port_);
    arduino_.SetBaudRate(LibSerial::BaudRate::BAUD_115200);

  }

  ~SimpleSerialReceiver(){
    arduino_.Close();
  }



  void timerCallback()
  {
    auto message = std_msgs::msg::String();

    if(rclcpp::ok()&&arduino_.IsDataAvailable()){
       arduino_.ReadLine(message.data);
    }
    
    pub_->publish(message);
  }

private:
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  LibSerial::SerialPort arduino_;
};


int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SimpleSerialReceiver>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}