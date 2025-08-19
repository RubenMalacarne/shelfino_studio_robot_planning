#include "planning_pkg/dubins_service.hpp"

DubinService::DubinService(): Node("dubins_service_node")
{
  //service
service_ = this->create_service<std_srvs::srv::Trigger>(
    "dubins_service",
    std::bind(&DubinService::handle_trigger, this, std::placeholders::_1, std::placeholders::_2));

  RCLCPP_INFO(this->get_logger(), "dubins_service service ready on: /dubins_service");
}

void DubinService::handle_trigger(
  const std::shared_ptr<std_srvs::srv::Trigger::Request> ,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  RCLCPP_INFO(this->get_logger(), "Messaggio ricevuto");
  //handle dubins curve
  response->success = true;
  response->message = "Trigger eseguito correttamente";
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DubinService>());
  rclcpp::shutdown();
  return 0;
}
