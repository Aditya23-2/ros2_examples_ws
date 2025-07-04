#include <chrono>
#include <thread>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

/**
 * @author: Mehmet Kahraman
 * @date: 09.10.2023
 * @updated: [current date]
 * @about: Action server node for UR5 robot control
 **/

using namespace std::placeholders;
using FollowJointTrajAction = control_msgs::action::FollowJointTrajectory;
using GoalHandleFollowJointTraj = rclcpp_action::ServerGoalHandle<FollowJointTrajAction>;

class UR5ActionServer : public rclcpp::Node
{   
    public:
        UR5ActionServer() : Node("ur5_action_server")
        {   
            std::string action_topic = "/ur5_joint_trajectory_action";
            
            action_server_ = rclcpp_action::create_server<FollowJointTrajAction>(
                this, 
                action_topic,
                std::bind(&UR5ActionServer::handle_goal, this, _1, _2),
                std::bind(&UR5ActionServer::handle_cancel, this, _1),
                std::bind(&UR5ActionServer::handle_accepted, this, _1));

            RCLCPP_INFO(this->get_logger(), "UR5 Action server node is ready.");
        }

    private:
        rclcpp_action::Server<FollowJointTrajAction>::SharedPtr action_server_;

        rclcpp_action::GoalResponse handle_goal(
            const rclcpp_action::GoalUUID &uuid,
            std::shared_ptr<const FollowJointTrajAction::Goal> goal)
        {
            RCLCPP_INFO(this->get_logger(), "Received trajectory goal request");
            
            // Validate the goal
            if (goal->trajectory.joint_names.size() != 6) {
                RCLCPP_ERROR(this->get_logger(), "Invalid joint names size. Expected 6 joints for UR5.");
                return rclcpp_action::GoalResponse::REJECT;
            }
            
            if (goal->trajectory.points.empty()) {
                RCLCPP_ERROR(this->get_logger(), "No trajectory points provided.");
                return rclcpp_action::GoalResponse::REJECT;
            }
            
            (void)uuid;
            return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        }

        rclcpp_action::CancelResponse handle_cancel(
            const std::shared_ptr<GoalHandleFollowJointTraj> goal_handle)
        {
            RCLCPP_INFO(this->get_logger(), "Received request to cancel trajectory goal");
            (void)goal_handle;
            return rclcpp_action::CancelResponse::ACCEPT;
        }

        void handle_accepted(const std::shared_ptr<GoalHandleFollowJointTraj> goal_handle)
        {
            std::thread{std::bind(&UR5ActionServer::execute_trajectory, this, _1), goal_handle}.detach();
        }

        void execute_trajectory(const std::shared_ptr<GoalHandleFollowJointTraj> goal_handle)
        {
            RCLCPP_INFO(this->get_logger(), "Executing trajectory...");
            
            const auto goal = goal_handle->get_goal();
            auto feedback = std::make_shared<FollowJointTrajAction::Feedback>();
            auto result = std::make_shared<FollowJointTrajAction::Result>();
            
            rclcpp::Time start_time = this->now();
            rclcpp::Rate loop_rate(10);  // 10 Hz
            
            // Simulate trajectory execution (in a real implementation, you would interface with the robot)
            for (const auto& point : goal->trajectory.points) {
                if (goal_handle->is_canceling()) {
                    result->error_code = FollowJointTrajAction::Result::SUCCESSFUL;
                    goal_handle->canceled(result);
                    RCLCPP_INFO(this->get_logger(), "Trajectory execution canceled");
                    return;
                }
                
                // Update feedback
                feedback->actual = point;
                goal_handle->publish_feedback(feedback);
                
                RCLCPP_INFO(this->get_logger(), "Executing point with %zu positions", point.positions.size());
                
                loop_rate.sleep();
            }
            
            // Trajectory completed successfully
            result->error_code = FollowJointTrajAction::Result::SUCCESSFUL;
            goal_handle->succeed(result);
            RCLCPP_INFO(this->get_logger(), "Trajectory execution completed successfully");
        }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto action_server_node = std::make_shared<UR5ActionServer>();
    rclcpp::spin(action_server_node);
    rclcpp::shutdown();
    return 0;
}
