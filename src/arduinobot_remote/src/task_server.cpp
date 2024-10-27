#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <memory>
#include <thread>
#include <rclcpp_components/register_node_macro.hpp>
#include <arduinobot_msgs/action/arduinobot_task.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

using namespace std::placeholders;

namespace arduinobot_remote
{
    class TaskServer : public rclcpp::Node
    {
    public:
        explicit TaskServer(const rclcpp::NodeOptions &options = rclcpp::NodeOptions()) : Node("task_server", options)
        {
            action_server_ = rclcpp_action::create_server<arduinobot_msgs::action::ArduinobotTask>(
                this, 
                "task_server",
                std::bind(&TaskServer::goalCallback, this, _1, _2),
                std::bind(&TaskServer::cancelCallback, this, _1),
                std::bind(&TaskServer::acceptedCallback, this, _1)
            );

            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Task server started");
        }

    private:
        rclcpp_action::Server<arduinobot_msgs::action::ArduinobotTask>::SharedPtr action_server_;
        std::shared_ptr<moveit::planning_interface::MoveGroupInterface> arm_move_group_, gripper_move_group_;
        std::vector<double> arm_joint_goal_, gripper_joint_goal_;

        rclcpp_action::GoalResponse goalCallback(const rclcpp_action::GoalUUID uuid, std::shared_ptr<const arduinobot_msgs::action::ArduinobotTask::Goal> goal)
        {
            RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "Received goal request with task_number: " << goal->task_number);
            (void)uuid;
            return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        }

        void acceptedCallback(const std::shared_ptr<rclcpp_action::ServerGoalHandle<arduinobot_msgs::action::ArduinobotTask>> goal_handle)
        {
            std::thread{std::bind(&TaskServer::execute, this, _1), goal_handle}.detach();
        }

        void execute(const std::shared_ptr<rclcpp_action::ServerGoalHandle<arduinobot_msgs::action::ArduinobotTask>> goal_handle)
        {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Executing Goal.");
            
            if(!arm_move_group_)
            {
                arm_move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), "arm");
            }
            if (!gripper_move_group_)
            {
                gripper_move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), "gripper");
            }

            auto result = std::make_shared<arduinobot_msgs::action::ArduinobotTask::Result>();

            if(goal_handle->get_goal()->task_number == 0)
            {
                arm_joint_goal_ = {0.0, 0.0, 0.0};
                gripper_joint_goal_ = {-0.7, 0.7};
            }
            else if (goal_handle->get_goal()->task_number == 1)
            {
                arm_joint_goal_ = {-1.14, -0.6, -0.07};
                gripper_joint_goal_ = {0.0, 0.0};
            }
            else if (goal_handle->get_goal()->task_number == 2)
            {
                arm_joint_goal_ = {-1.57, 0.0, -0.9};
                gripper_joint_goal_ = {0.0, 0.0};
            }
            else
            {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Invalid Task number.");
                return;
            }

            arm_move_group_->setStartState(*arm_move_group_->getCurrentState());
            gripper_move_group_->setStartState(*gripper_move_group_->getCurrentState());

            bool arm_within_bounds = arm_move_group_->setJointValueTarget(arm_joint_goal_);
            bool gripper_within_bounds = gripper_move_group_->setJointValueTarget(gripper_joint_goal_);

            if (!arm_within_bounds || !gripper_within_bounds)
            {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Target position out of bounds.");
                return;
            }

            moveit::planning_interface::MoveGroupInterface::Plan arm_plan, gripper_plan;
            bool arm_plan_success = (arm_move_group_->plan(arm_plan) == moveit::core::MoveItErrorCode::SUCCESS);
            bool gripper_plan_success = (gripper_move_group_->plan(gripper_plan) == moveit::core::MoveItErrorCode::SUCCESS);

            if (arm_plan_success && gripper_plan_success)
            {
                arm_move_group_->move();
                gripper_move_group_->move();
            }
            else
            {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "One or more planners failed.");
                return;
            }

            result->success = true;
            goal_handle->succeed(result);
        }

        rclcpp_action::CancelResponse cancelCallback(const std::shared_ptr<rclcpp_action::ServerGoalHandle<arduinobot_msgs::action::ArduinobotTask>> goal_handle)
        {
            if(arm_move_group_)
            {
                arm_move_group_->stop();
            }
            if(gripper_move_group_)
            {
                gripper_move_group_->stop();
            }
            (void)goal_handle;
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Received request cancel the goal. ");
            return rclcpp_action::CancelResponse::ACCEPT;
        }
    };

}

RCLCPP_COMPONENTS_REGISTER_NODE(arduinobot_remote::TaskServer)