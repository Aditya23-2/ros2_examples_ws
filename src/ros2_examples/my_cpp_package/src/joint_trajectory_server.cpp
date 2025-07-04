#include <chrono>
#include <thread>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

using namespace std::placeholders;
using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
using GoalHandle = rclcpp_action::ServerGoalHandle<FollowJointTrajectory>;

class JointTrajectoryActionServer : public rclcpp::Node
{   
public:
    JointTrajectoryActionServer() : Node("joint_trajectory_controller")
    {   
        action_server_ = rclcpp_action::create_server<FollowJointTrajectory>(
            this, 
            "/joint_trajectory_controller/follow_joint_trajectory",
            std::bind(&JointTrajectoryActionServer::handle_goal, this, _1, _2),
            std::bind(&JointTrajectoryActionServer::handle_cancel, this, _1),
            std::bind(&JointTrajectoryActionServer::handle_accepted, this, _1));

        RCLCPP_INFO(this->get_logger(), "Joint Trajectory Action Server ready.");
    }

private:
    rclcpp_action::Server<FollowJointTrajectory>::SharedPtr action_server_;

    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID &uuid,
        std::shared_ptr<const FollowJointTrajectory::Goal> goal)
    {
        RCLCPP_INFO(this->get_logger(), "Received trajectory goal with %zu points", 
                   goal->trajectory.points.size());
        (void)uuid;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandle> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(const std::shared_ptr<GoalHandle> goal_handle)
    {
        std::thread{std::bind(&JointTrajectoryActionServer::execute_trajectory, this, _1), goal_handle}.detach();
    }

    void execute_trajectory(const std::shared_ptr<GoalHandle> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Executing trajectory...");
        
        const auto goal = goal_handle->get_goal();
        auto feedback = std::make_shared<FollowJointTrajectory::Feedback>();
        auto result = std::make_shared<FollowJointTrajectory::Result>();

        // Simulate trajectory execution
        rclcpp::Rate loop_rate(10);
        size_t current_point = 0;
        const size_t total_points = goal->trajectory.points.size();

        while (rclcpp::ok() && current_point < total_points) {
            if (goal_handle->is_canceling()) {
                result->error_code = FollowJointTrajectory::Result::PATH_TOLERANCE_VIOLATED;
                goal_handle->canceled(result);
                RCLCPP_INFO(this->get_logger(), "Trajectory canceled");
                return;
            }

            // Update feedback
            feedback->actual = goal->trajectory.points[current_point];
            feedback->desired = goal->trajectory.points[current_point];
            feedback->error.positions.assign(
                goal->trajectory.joint_names.size(), 0.0);
            goal_handle->publish_feedback(feedback);

            current_point++;
            loop_rate.sleep();
        }

        if (rclcpp::ok()) {
            result->error_code = FollowJointTrajectory::Result::SUCCESSFUL;
            goal_handle->succeed(result);
            RCLCPP_INFO(this->get_logger(), "Trajectory executed successfully");
        }
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<JointTrajectoryActionServer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
