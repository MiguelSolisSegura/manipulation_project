#include <cmath>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include "geometry_msgs/msg/detail/pose__struct.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"
#include "rclcpp/logger.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/node_options.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/time.hpp"
#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <thread>
#include <vector>
#include <chrono>
#include "grasping_msgs/action/find_graspable_objects.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include <inttypes.h>
#include <iostream>
#include <memory>
#include <string>
using namespace std::chrono_literals;

class GetPoseClient : public rclcpp::Node {
public:
    using Find = grasping_msgs::action::FindGraspableObjects;
    using GoalHandleFind = rclcpp_action::ClientGoalHandle<Find>;

    explicit GetPoseClient(
        const rclcpp::NodeOptions &node_options = rclcpp::NodeOptions())
        : Node("get_pose_client", node_options), goal_done_(false) {
        this->client_ptr_ = rclcpp_action::create_client<Find>(
            this->get_node_base_interface(), this->get_node_graph_interface(),
            this->get_node_logging_interface(),
            this->get_node_waitables_interface(), "find_objects");

        this->timer_ =
            this->create_wall_timer(std::chrono::milliseconds(500),
                                    std::bind(&GetPoseClient::send_goal, this));
    }

    bool is_goal_done() const { return this->goal_done_; }

    void send_goal() {
        using namespace std::placeholders;

        this->timer_->cancel();

        this->goal_done_ = false;

        if (!this->client_ptr_) {
        RCLCPP_ERROR(this->get_logger(), "Action client not initialized");
        }

        if (!this->client_ptr_->wait_for_action_server(std::chrono::seconds(15))) {
        RCLCPP_ERROR(this->get_logger(),
                    "Action server not available after waiting");
        this->goal_done_ = true;
        return;
        }

        auto goal_msg = Find::Goal();
        goal_msg.plan_grasps = false;

        RCLCPP_INFO(this->get_logger(), "Sending goal");

        auto send_goal_options = rclcpp_action::Client<Find>::SendGoalOptions();
        send_goal_options.goal_response_callback =
            std::bind(&GetPoseClient::goal_response_callback, this, _1);
        send_goal_options.feedback_callback =
            std::bind(&GetPoseClient::feedback_callback, this, _1, _2);
        send_goal_options.result_callback =
            std::bind(&GetPoseClient::result_callback, this, _1);
        auto goal_handle_future =
            this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
    }

private:
    rclcpp_action::Client<Find>::SharedPtr client_ptr_;
    rclcpp::TimerBase::SharedPtr timer_;
    bool goal_done_;
    float x_;
    float y_;

    void goal_response_callback(const GoalHandleFind::SharedPtr &goal_handle) {
        if (!goal_handle) {
        RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
        } else {
        RCLCPP_INFO(this->get_logger(),
                    "Goal accepted by server, waiting for result");
        }
    }

    void feedback_callback(GoalHandleFind::SharedPtr,
                            const std::shared_ptr<const Find::Feedback>) {
        RCLCPP_INFO(this->get_logger(), "Ignoring feedback...");
    }

    void result_callback(const GoalHandleFind::WrappedResult &result) {
        this->goal_done_ = true;
        switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
        break;
        case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
        return;
        case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
        return;
        default:
        RCLCPP_ERROR(this->get_logger(), "Unknown result code");
        return;
        }

        RCLCPP_INFO(this->get_logger(), "Result received");
        x_ = result.result->objects[0].object.primitive_poses[0].position.x;
        y_ = result.result->objects[0].object.primitive_poses[0].position.y;
        RCLCPP_INFO(this->get_logger(), "X: %f", x_);
        RCLCPP_INFO(this->get_logger(), "Y: %f", y_);
        pick_and_place();
    }

    void setTarget(std::string name, geometry_msgs::msg::Pose &pose, const std::vector<double> &target) {
        RCLCPP_INFO(this->get_logger(), "New target: %s.", name.c_str());
        pose.position.x = target[0];      
        pose.position.y = target[1];
        pose.position.z = target[2];
        pose.orientation.x = target[3];
        pose.orientation.y = target[4];
        pose.orientation.z = target[5];
        pose.orientation.w = target[6];
        return;
    }

    void pick_and_place() {
        // Node configuration
        rclcpp::NodeOptions node_options;
        node_options.automatically_declare_parameters_from_overrides(true);
        auto move_group_node = rclcpp::Node::make_shared("move_group_pick_and_place", node_options);
        // Thread configuration
        rclcpp::executors::SingleThreadedExecutor executor;
        executor.add_node(move_group_node);
        std::thread([&executor](){executor.spin();}).detach();
        // Definition of pose targets: position and orientation
        const std::vector<double> PRE_GRASP_POSE{x_, y_, 0.3, -1.0, 0.00, 0.00, 0.00};
        const std::vector<double> GRASP_POSE{x_, y_, 0.2, -1.0, 0.00, 0.00, 0.00};
        // Planning interfaces
        static const std::string PLANNING_GROUP_ARM = "ur_manipulator";
        static const std::string PLANNING_GROUP_GRIPPER = "gripper";
        moveit::planning_interface::MoveGroupInterface move_group_arm(move_group_node, PLANNING_GROUP_ARM);
        moveit::planning_interface::MoveGroupInterface move_group_gripper(move_group_node, PLANNING_GROUP_GRIPPER);
        // Joint models
        const moveit::core::JointModelGroup *joint_model_group_arm = move_group_arm.getCurrentState()->getJointModelGroup(PLANNING_GROUP_ARM);
        const moveit::core::JointModelGroup *joint_model_group_gripper = move_group_gripper.getCurrentState()->getJointModelGroup(PLANNING_GROUP_GRIPPER);
        // Get current state
        moveit::core::RobotStatePtr current_state_arm = move_group_arm.getCurrentState(10);
        moveit::core::RobotStatePtr current_state_gripper = move_group_gripper.getCurrentState(10);
        // Joit positions
        std::vector<double> joint_group_positions_arm;
        std::vector<double> joint_group_positions_gripper;
        current_state_arm->copyJointGroupPositions(joint_model_group_arm, joint_group_positions_arm);
        current_state_gripper->copyJointGroupPositions(joint_model_group_gripper, joint_group_positions_gripper);
        // Start state
        move_group_arm.setStartStateToCurrentState();
        move_group_gripper.setStartStateToCurrentState();
        // Planning interfaces
        moveit::planning_interface::MoveGroupInterface::Plan plan_arm;
        moveit::planning_interface::MoveGroupInterface::Plan plan_gripper;
        // Pose target for development
        geometry_msgs::msg::Pose target_pose;
        // Sucess flags
        bool success_arm;
        bool success_gripper;

        // Create collision object for the robot to avoid
        auto const collision_object = [frame_id =
                                        move_group_arm.getPlanningFrame()] {
            moveit_msgs::msg::CollisionObject collision_object;
            collision_object.header.frame_id = frame_id;
            collision_object.id = "box1";
            shape_msgs::msg::SolidPrimitive primitive_horizontal;
            shape_msgs::msg::SolidPrimitive primitive_vertical;

            primitive_horizontal.type = primitive_horizontal.BOX;
            primitive_horizontal.dimensions.resize(3);
            primitive_horizontal.dimensions[primitive_horizontal.BOX_X] = 10.0;
            primitive_horizontal.dimensions[primitive_horizontal.BOX_Y] = 10.0;
            primitive_horizontal.dimensions[primitive_horizontal.BOX_Z] = 0.05;

            primitive_vertical.type = primitive_vertical.BOX;
            primitive_vertical.dimensions.resize(3);
            primitive_vertical.dimensions[primitive_vertical.BOX_X] = 10.0;
            primitive_vertical.dimensions[primitive_vertical.BOX_Y] = 0.05;
            primitive_vertical.dimensions[primitive_vertical.BOX_Z] = 10.0;

            // Define the pose relative to the frame_id
            geometry_msgs::msg::Pose pose_horizontal;
            pose_horizontal.orientation.w = 1.0; 
            pose_horizontal.position.x = 0.0;
            pose_horizontal.position.y = 0.0;
            pose_horizontal.position.z = -0.025;

            geometry_msgs::msg::Pose pose_vertical;
            pose_vertical.orientation.w = 1.0; 
            pose_vertical.position.x = 0.0;
            pose_vertical.position.y = 0.5;
            pose_vertical.position.z = 0.0;

            collision_object.primitives.push_back(primitive_horizontal);
            collision_object.primitive_poses.push_back(pose_horizontal);
            collision_object.primitives.push_back(primitive_vertical);
            collision_object.primitive_poses.push_back(pose_vertical);
            collision_object.operation = collision_object.ADD;

            return collision_object;
        }();

        moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
        planning_scene_interface.applyCollisionObject(collision_object);


        // Open gripper 
        RCLCPP_INFO(this->get_logger(), "Opening gripper.");
        move_group_gripper.setNamedTarget("gripper_open");
        success_gripper = (move_group_gripper.plan(plan_gripper) == moveit::core::MoveItErrorCode::SUCCESS);
        if (success_gripper) {
            RCLCPP_INFO(this->get_logger(), "Plan to actuate gipper: SUCCESS.");
            RCLCPP_INFO(this->get_logger(), "Executing command.");
            move_group_arm.execute(plan_gripper);
        } else {
            RCLCPP_INFO(this->get_logger(), "Plan to actuate gipper: FAIL.");
            RCLCPP_INFO(this->get_logger(), "Aborting command.");
            rclcpp::shutdown();
            return;
        }

        // Pre-grasp position
        setTarget("pre_grasp", target_pose, PRE_GRASP_POSE);
        move_group_arm.setPoseTarget(target_pose);
        success_arm = (move_group_arm.plan(plan_arm) == moveit::core::MoveItErrorCode::SUCCESS);
        if (success_arm) {
            RCLCPP_INFO(this->get_logger(), "Plan to target status: SUCCESS.");
            RCLCPP_INFO(this->get_logger(), "Executing motion.");
            move_group_arm.execute(plan_arm);
        } else {
            RCLCPP_INFO(this->get_logger(), "Plan to target status: FAIL.");
            RCLCPP_INFO(this->get_logger(), "Aborting motion.");
            rclcpp::shutdown();
            return;
        }

        // Grasp trajectory
        std::vector<geometry_msgs::msg::Pose> grasp_waypoints;
        std::vector<geometry_msgs::msg::Pose> reverse_grasp_waypoints;
        grasp_waypoints.push_back(target_pose);
        setTarget("grasp", target_pose, GRASP_POSE);
        grasp_waypoints.push_back(target_pose);
        reverse_grasp_waypoints.push_back(target_pose);
        setTarget("pre_grasp", target_pose, PRE_GRASP_POSE);
        reverse_grasp_waypoints.push_back(target_pose);


        moveit_msgs::msg::RobotTrajectory trajectory_approach;
        const double jump_threshold = 0.0;
        const double eef_step = 0.01;

        double fraction = move_group_arm.computeCartesianPath(grasp_waypoints, eef_step, jump_threshold, trajectory_approach);
        RCLCPP_INFO(this->get_logger(), "Fraction: %.3f.", float(fraction));
        move_group_arm.execute(trajectory_approach);

        // Close gripper
        float gripper_value = 0.65;
        joint_group_positions_gripper[2] = gripper_value;
        move_group_gripper.setJointValueTarget(joint_group_positions_gripper);
        RCLCPP_INFO(this->get_logger(), "Closing gripper: %.3f.", gripper_value);
        success_gripper = (move_group_gripper.plan(plan_gripper) == moveit::core::MoveItErrorCode::SUCCESS);
        if (success_gripper) {
            RCLCPP_INFO(this->get_logger(), "Plan to actuate gipper: SUCCESS.");
            RCLCPP_INFO(this->get_logger(), "Executing command.");
            move_group_arm.execute(plan_gripper);
            gripper_value += 0.001;
        } else {
            RCLCPP_INFO(this->get_logger(), "Plan to actuate gipper: FAIL.");
            RCLCPP_INFO(this->get_logger(), "Aborting command.");
            rclcpp::shutdown();
            return;
        }

        fraction = move_group_arm.computeCartesianPath(reverse_grasp_waypoints, eef_step, jump_threshold, trajectory_approach);
        RCLCPP_INFO(this->get_logger(), "Fraction: %.3f.", float(fraction));
        move_group_arm.execute(trajectory_approach);

        // Fetch the most recent state
        current_state_arm = move_group_arm.getCurrentState(10);
        current_state_arm->copyJointGroupPositions(joint_model_group_arm, joint_group_positions_arm);

        // Rotating the first joint 180 degrees
        joint_group_positions_arm[0] -= M_PI;
        move_group_arm.setJointValueTarget(joint_group_positions_arm);
        success_arm = (move_group_arm.plan(plan_arm) == moveit::core::MoveItErrorCode::SUCCESS);
        if (success_arm) {
            RCLCPP_INFO(this->get_logger(), "Plan to rotate first joint 180 degrees: SUCCESS.");
            RCLCPP_INFO(this->get_logger(), "Executing rotation.");
            move_group_arm.execute(plan_arm);
        } else {
            RCLCPP_INFO(this->get_logger(), "Plan to rotate first joint 180 degrees: FAIL.");
            RCLCPP_INFO(this->get_logger(), "Aborting rotation.");
            rclcpp::shutdown();
            return;
        }

        // Open gripper
        RCLCPP_INFO(this->get_logger(), "Opening gripper.");
        move_group_gripper.setNamedTarget("gripper_open");
        success_gripper = (move_group_gripper.plan(plan_gripper) == moveit::core::MoveItErrorCode::SUCCESS);
        if (success_gripper) {
            RCLCPP_INFO(this->get_logger(), "Plan to actuate gipper: SUCCESS.");
            RCLCPP_INFO(this->get_logger(), "Executing command.");
            move_group_arm.execute(plan_gripper);
        } else {
            RCLCPP_INFO(this->get_logger(), "Plan to actuate gipper: FAIL.");
            RCLCPP_INFO(this->get_logger(), "Aborting command.");
            rclcpp::shutdown();
            return;
        }
        rclcpp::shutdown();
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto action_client = std::make_shared<GetPoseClient>();

    while (!action_client->is_goal_done()) {
        rclcpp::spin_some(action_client);
    }

    rclcpp::shutdown();
    return 0;
}