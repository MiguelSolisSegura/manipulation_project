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
using namespace std::chrono_literals;

// Definition of joint targets: shoulder to twrist
const std::vector<double> HOME{0.00, -2.50, 1.50, -1.50, -1.55, 0.00};
const std::vector<double> PRE_GRASP{0.0029, -1.3741, 1.5374, -1.7348, -1.5707, -1.5689};
const std::vector<double> GRASP{0.0028, -1.2934, 1.7496, -2.0263, -1.5701, -1.5679};
const std::vector<double> DROP{-3.1387, -1.3741, 1.5374, -1.7348, -1.5707, -1.5689};


// Definition of pose targets: position and orientation
const std::vector<double> PRE_GRASP_POSE{0.343, 0.132, 0.300, -1.0, 0.00, 0.00, 0.00};
const std::vector<double> GRASP_POSE{0.343, 0.132, 0.200, -1.0, 0.00, 0.00, 0.00};

static const rclcpp::Logger LOGGER = rclcpp::get_logger("pick_and_place");

void setTarget(std::string name, std::vector<double> &joint_group_positions, const std::vector<double> &target) {
    RCLCPP_INFO(LOGGER, "New target: %s.", name.c_str());
    joint_group_positions[0] = target[0];   // Shoulder Pan
    joint_group_positions[1] = target[1];   // Shoulder Lift
    joint_group_positions[2] = target[2];   // Elbow
    joint_group_positions[3] = target[3];   // Wrist 1
    joint_group_positions[4] = target[4];   // Wrist 2
    joint_group_positions[5] = target[5];   // Wrist 3
    return;
}

void setTarget(std::string name, geometry_msgs::msg::Pose &pose, const std::vector<double> &target) {
    RCLCPP_INFO(LOGGER, "New target: %s.", name.c_str());
    pose.position.x = target[0];      
    pose.position.y = target[1];
    pose.position.z = target[2];
    pose.orientation.x = target[3];
    pose.orientation.y = target[4];
    pose.orientation.z = target[5];
    pose.orientation.w = target[6];
    return;
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    // Node options
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    auto move_group_node = rclcpp::Node::make_shared("move_group_pick_and_place", node_options);
    // Thread configuration
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(move_group_node);
    std::thread([&executor](){executor.spin();}).detach();
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
    RCLCPP_INFO(LOGGER, "Opening gripper.");
    move_group_gripper.setNamedTarget("gripper_open");
    success_gripper = (move_group_gripper.plan(plan_gripper) == moveit::core::MoveItErrorCode::SUCCESS);
    if (success_gripper) {
        RCLCPP_INFO(LOGGER, "Plan to actuate gipper: SUCCESS.");
        RCLCPP_INFO(LOGGER, "Executing command.");
        move_group_arm.execute(plan_gripper);
    } else {
        RCLCPP_INFO(LOGGER, "Plan to actuate gipper: FAIL.");
        RCLCPP_INFO(LOGGER, "Aborting command.");
        rclcpp::shutdown();
        return 1;
    }

    // Pre-grasp position
    setTarget("pre_grasp", target_pose, PRE_GRASP_POSE);
    move_group_arm.setPoseTarget(target_pose);
    success_arm = (move_group_arm.plan(plan_arm) == moveit::core::MoveItErrorCode::SUCCESS);
    if (success_arm) {
        RCLCPP_INFO(LOGGER, "Plan to target status: SUCCESS.");
        RCLCPP_INFO(LOGGER, "Executing motion.");
        move_group_arm.execute(plan_arm);
    } else {
        RCLCPP_INFO(LOGGER, "Plan to target status: FAIL.");
        RCLCPP_INFO(LOGGER, "Aborting motion.");
        rclcpp::shutdown();
        return 1;
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
    RCLCPP_INFO(LOGGER, "Fraction: %.3f.", float(fraction));
    move_group_arm.execute(trajectory_approach);

    // Close gripper
    float gripper_value = 0.65;
    joint_group_positions_gripper[2] = gripper_value;
    move_group_gripper.setJointValueTarget(joint_group_positions_gripper);
    RCLCPP_INFO(LOGGER, "Closing gripper: %.3f.", gripper_value);
    success_gripper = (move_group_gripper.plan(plan_gripper) == moveit::core::MoveItErrorCode::SUCCESS);
    if (success_gripper) {
        RCLCPP_INFO(LOGGER, "Plan to actuate gipper: SUCCESS.");
        RCLCPP_INFO(LOGGER, "Executing command.");
        move_group_arm.execute(plan_gripper);
        gripper_value += 0.001;
    } else {
        RCLCPP_INFO(LOGGER, "Plan to actuate gipper: FAIL.");
        RCLCPP_INFO(LOGGER, "Aborting command.");
        rclcpp::shutdown();
        return 1;
    }

    fraction = move_group_arm.computeCartesianPath(reverse_grasp_waypoints, eef_step, jump_threshold, trajectory_approach);
    RCLCPP_INFO(LOGGER, "Fraction: %.3f.", float(fraction));
    move_group_arm.execute(trajectory_approach);

    // Fetch the most recent state
    current_state_arm = move_group_arm.getCurrentState(10);
    current_state_arm->copyJointGroupPositions(joint_model_group_arm, joint_group_positions_arm);

    // Rotating the first joint 180 degrees
    joint_group_positions_arm[0] -= M_PI;
    move_group_arm.setJointValueTarget(joint_group_positions_arm);
    success_arm = (move_group_arm.plan(plan_arm) == moveit::core::MoveItErrorCode::SUCCESS);
    if (success_arm) {
        RCLCPP_INFO(LOGGER, "Plan to rotate first joint 180 degrees: SUCCESS.");
        RCLCPP_INFO(LOGGER, "Executing rotation.");
        move_group_arm.execute(plan_arm);
    } else {
        RCLCPP_INFO(LOGGER, "Plan to rotate first joint 180 degrees: FAIL.");
        RCLCPP_INFO(LOGGER, "Aborting rotation.");
        rclcpp::shutdown();
        return 1;
    }

    // Open gripper
    RCLCPP_INFO(LOGGER, "Opening gripper.");
    move_group_gripper.setNamedTarget("gripper_open");
    success_gripper = (move_group_gripper.plan(plan_gripper) == moveit::core::MoveItErrorCode::SUCCESS);
    if (success_gripper) {
        RCLCPP_INFO(LOGGER, "Plan to actuate gipper: SUCCESS.");
        RCLCPP_INFO(LOGGER, "Executing command.");
        move_group_arm.execute(plan_gripper);
    } else {
        RCLCPP_INFO(LOGGER, "Plan to actuate gipper: FAIL.");
        RCLCPP_INFO(LOGGER, "Aborting command.");
        rclcpp::shutdown();
        return 1;
    }

    rclcpp::shutdown();
    return 0;  
}