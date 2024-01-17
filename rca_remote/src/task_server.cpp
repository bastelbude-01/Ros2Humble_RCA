#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <rca_msgs/action/rca_task.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

#include <memory>
#include <thread>

using namespace std::placeholders;

namespace rca_remote
{
  class RcaTaskServer : public rclcpp::Node
  {
  public:
    explicit RcaTaskServer(const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
        : Node("rca_task_server", options)
    {
      RCLCPP_INFO(get_logger(), "Starting the RCA Task Server");
      action_server_ = rclcpp_action::create_server<rca_msgs::action::RcaTask>(
          this, "rca_task_server", std::bind(&RcaTaskServer::goalCallback, this, _1, _2),
          std::bind(&RcaTaskServer::cancelCallback, this, _1),
          std::bind(&RcaTaskServer::acceptedCallback, this, _1));
    }

  private:
    rclcpp_action::Server<rca_msgs::action::RcaTask>::SharedPtr action_server_;

    rclcpp_action::GoalResponse goalCallback(
        const rclcpp_action::GoalUUID &uuid,
        std::shared_ptr<const rca_msgs::action::RcaTask::Goal> goal)
    {
      RCLCPP_INFO(get_logger(), "Received goal request with id %d", goal->task_number);
      (void)uuid;
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse cancelCallback(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<rca_msgs::action::RcaTask>> goal_handle)
    {
      RCLCPP_INFO(get_logger(), "Received request to cancel goal");

      auto arm_move_group = moveit::planning_interface::MoveGroupInterface(shared_from_this(), "arm");
      auto gripper_move_group = moveit::planning_interface::MoveGroupInterface(shared_from_this(), "gripper");

      arm_move_group.stop();
      gripper_move_group.stop();

      return rclcpp_action::CancelResponse::ACCEPT;
    }

    void acceptedCallback(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<rca_msgs::action::RcaTask>> goal_handle)
    {
      std::thread{std::bind(&RcaTaskServer::execute, this, _1), goal_handle}.detach();
    }

    void execute(const std::shared_ptr<rclcpp_action::ServerGoalHandle<rca_msgs::action::RcaTask>> goal_handle)
    {
      RCLCPP_INFO(get_logger(), "Executing goal");

      auto arm_move_group = moveit::planning_interface::MoveGroupInterface(shared_from_this(), "arm");
      auto gripper_move_group = moveit::planning_interface::MoveGroupInterface(shared_from_this(), "gripper");

      std::vector<double> arm_joint_goal;
      std::vector<double> gripper_joint_goal;

      if (goal_handle->get_goal()->task_number == 0)
      {
        arm_joint_goal = {0.0, 0.0, 0.0, 0.0, 0.0};
        gripper_joint_goal = { -0.3, 0.3 };
      }
      else if (goal_handle->get_goal()->task_number == 1)
      {
        arm_joint_goal = {0.6, 0.0, 0.0, 0.0, 0.0};
        gripper_joint_goal = { -0.3, 0.3 };
      }
      else if (goal_handle->get_goal()->task_number == 2)
      {
        arm_joint_goal = {0.6, 0.3, 0.2, 0.0, 0.0};
        gripper_joint_goal = { 0.0, 0.0 };
      }
      else
      {
        RCLCPP_ERROR(get_logger(), "Falsche Task Nummer");
        return;
      }

      bool arm_within_bounds = arm_move_group.setJointValueTarget(arm_joint_goal);
      bool gripper_within_bounds = gripper_move_group.setJointValueTarget(gripper_joint_goal);

      if(!arm_within_bounds | !gripper_within_bounds)
      {
        RCLCPP_WARN(get_logger(),"Ziel Joint Position ausserhalb der Limits");
        return;
      }

      moveit::planning_interface::MoveGroupInterface::Plan arm_plan;
      moveit::planning_interface::MoveGroupInterface::Plan gripper_plan;

      bool arm_plan_success = arm_move_group.plan(arm_plan) == moveit::core::MoveItErrorCode::SUCCESS;
      bool gripper_plan_success = gripper_move_group.plan(gripper_plan)== moveit::core::MoveItErrorCode::SUCCESS;

      if(arm_plan_success && gripper_plan_success)      
      {
        RCLCPP_INFO(get_logger(),"Planung Erfolgreich, Arm in Bewegung");

        arm_move_group.move();
        gripper_move_group.move();
        
      }
      else
      {
        RCLCPP_ERROR(get_logger(),"Eine oder mehrere Berechnungen sind Fehlgeschlagen");
        return;
      }

      auto result = std::make_shared<rca_msgs::action::RcaTask::Result>();
      result->success = true;
      goal_handle->succeed(result);
      RCLCPP_INFO(get_logger(),"Task Ausgefuehrt");
    }
  };
} 

RCLCPP_COMPONENTS_REGISTER_NODE(rca_remote::RcaTaskServer)