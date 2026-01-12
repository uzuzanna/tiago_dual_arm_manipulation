#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp> 
#include <rclcpp/qos.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp> 
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <cmath>
#include <linkattacher_msgs/srv/attach_link.hpp>
#include <linkattacher_msgs/srv/detach_link.hpp>


using namespace std::chrono_literals;

class MoveToTarget : public rclcpp::Node
{
public:
    using GripperAction = control_msgs::action::FollowJointTrajectory;
    using GoalHandleGripper = rclcpp_action::ClientGoalHandle<GripperAction>;

    MoveToTarget(const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
        : Node("move_to_target_node", options)
    {
        callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
        RCLCPP_INFO(this->get_logger(), "Init Dual Arm Control...");

        gripper_right_client_ = rclcpp_action::create_client<GripperAction>(
            this, "/gripper_right_controller/follow_joint_trajectory", callback_group_);

        gripper_left_client_ = rclcpp_action::create_client<GripperAction>(
            this, "/gripper_left_controller/follow_joint_trajectory", callback_group_);

        attach_client_ = this->create_client<linkattacher_msgs::srv::AttachLink>("/ATTACHLINK", rmw_qos_profile_services_default, callback_group_);
        detach_client_ = this->create_client<linkattacher_msgs::srv::DetachLink>("/DETACHLINK", rmw_qos_profile_services_default, callback_group_);
    }

    void initMoveGroups()
    {
        right_arm_torso_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
            shared_from_this(), "arm_right_torso");
        configureGroup(right_arm_torso_group_, "arm_right_tool_link");

        left_arm_no_torso_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
            shared_from_this(), "arm_left");
        configureGroup(left_arm_no_torso_group_, "arm_left_tool_link");

        left_arm_torso_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
            shared_from_this(), "arm_left_torso");
        configureGroup(left_arm_torso_group_, "arm_left_tool_link");
        
        right_arm_no_torso_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
            shared_from_this(), "arm_right");
        configureGroup(right_arm_no_torso_group_, "arm_right_tool_link");
        
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        
        RCLCPP_INFO(this->get_logger(), "MoveGroups Ready: Right+Torso, Left_No_Torso, Left+Torso.");

        rclcpp::SubscriptionOptions sub_options;
        sub_options.callback_group = callback_group_;
        sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/target_pose", 10, 
            std::bind(&MoveToTarget::poseCallback, this, std::placeholders::_1),
            sub_options);
    }

private:
    void configureGroup(std::shared_ptr<moveit::planning_interface::MoveGroupInterface> group, std::string link) {
        group->setPoseReferenceFrame("base_link");
        group->setEndEffectorLink(link);
        group->setPlanningTime(20.0);
        group->setGoalPositionTolerance(0.005);
        group->setGoalOrientationTolerance(0.005); 
        group->setMaxVelocityScalingFactor(0.8);
        group->setMaxAccelerationScalingFactor(0.8);
        group->startStateMonitor();
    }
    
    void driveBackwards()
    {
        RCLCPP_INFO(this->get_logger(), ">>> STARTING DRIVE BACKWARDS <<<");
        
        geometry_msgs::msg::Twist msg;
        msg.linear.x = -0.5;
        msg.angular.z = 0.0;

        rclcpp::Rate loop_rate(10);
        
        for(int i=0; i<20; i++) {
            cmd_vel_pub_->publish(msg);
            loop_rate.sleep();
        }

        msg.linear.x = 0.0;
        cmd_vel_pub_->publish(msg);
        
        RCLCPP_INFO(this->get_logger(), ">>> DRIVE FINISHED <<<");
        rclcpp::sleep_for(std::chrono::milliseconds(1000)); 
    }
    
    void driveForward()
    {
        RCLCPP_INFO(this->get_logger(), ">>> STARTING DRIVE <<<");
        
        geometry_msgs::msg::Twist msg;
        msg.linear.x = 0.5;
        msg.angular.z = 0.0;

        rclcpp::Rate loop_rate(10);
        
        for(int i=0; i<20; i++) {
            cmd_vel_pub_->publish(msg);
            loop_rate.sleep();
        }

        msg.linear.x = 0.0;
        cmd_vel_pub_->publish(msg);
        
        RCLCPP_INFO(this->get_logger(), ">>> DRIVE FINISHED <<<");
        rclcpp::sleep_for(std::chrono::milliseconds(1000)); 
    }

    bool attachCube(std::string side)
    {
        if (!attach_client_->wait_for_service(std::chrono::seconds(2))) {
            RCLCPP_ERROR(this->get_logger(), "Service /ATTACHLINK not available!");
            return false;
        }

        auto request = std::make_shared<linkattacher_msgs::srv::AttachLink::Request>();
        request->model1_name = "tiago-pro";
        
        if (side == "left") {
            request->link1_name = "arm_left_7_link";
        } 
        else if (side == "right") {
            request->link1_name = "arm_right_7_link";
        }

        request->model2_name = "rubiks_cube";
        request->link2_name = "link";

        RCLCPP_INFO(this->get_logger(), "Attaching cube to %s arm...", side.c_str());
        
        auto future = attach_client_->async_send_request(request);
        if (future.wait_for(std::chrono::seconds(2)) == std::future_status::ready) {
            auto result = future.get();
            if (result->success) {
                RCLCPP_INFO(this->get_logger(), ">>> ATTACH %s SUCCESS! <<<", side.c_str());
                return true;
            } else {
                RCLCPP_ERROR(this->get_logger(), ">>> ATTACH %s FAILED (Plugin returned false) <<<", side.c_str());
            }
        } else {
            RCLCPP_ERROR(this->get_logger(), ">>> ATTACH %s TIMEOUT <<<", side.c_str());
        }
        return false;
    }

    bool detachCube(std::string side)
    {
        if (!detach_client_->wait_for_service(std::chrono::seconds(2))) {
            RCLCPP_ERROR(this->get_logger(), "Service /DETACHLINK not available!");
            return false;
        }

        auto request = std::make_shared<linkattacher_msgs::srv::DetachLink::Request>();
        request->model1_name = "tiago-pro";
        
        if (side == "left") {
            request->link1_name = "arm_left_7_link";
        }
        else if (side == "right") {
            request->link1_name = "arm_right_7_link";
        }

        request->model2_name = "rubiks_cube";
        request->link2_name = "link";

        RCLCPP_INFO(this->get_logger(), "Detaching cube from %s arm...", side.c_str());

        auto future = detach_client_->async_send_request(request);
        if (future.wait_for(std::chrono::seconds(2)) == std::future_status::ready) {
            if (future.get()->success) {
                RCLCPP_INFO(this->get_logger(), ">>> DETACHED %s SUCCESS! <<<", side.c_str());
                return true;
            }
        }
        RCLCPP_ERROR(this->get_logger(), "Detach %s failed!", side.c_str());
        return false;
    }

    bool operateGripperGeneric(rclcpp_action::Client<GripperAction>::SharedPtr client, 
                               std::string joint_name, double position)
    {
        if (!client->wait_for_action_server(std::chrono::seconds(2))) {
            RCLCPP_ERROR(this->get_logger(), "Gripper action server not available");
            return false;
        }

        auto goal_msg = GripperAction::Goal();
        goal_msg.trajectory.joint_names.push_back(joint_name);

        trajectory_msgs::msg::JointTrajectoryPoint point;
        point.positions.push_back(position); 
        point.time_from_start = rclcpp::Duration::from_seconds(1.0);
        goal_msg.trajectory.points.push_back(point);

        control_msgs::msg::JointTolerance tolerance;
        tolerance.name = joint_name;
        tolerance.position = 0.1;  
        tolerance.velocity = 1.0;
        tolerance.acceleration = 1.0;
        goal_msg.goal_tolerance.push_back(tolerance);
        goal_msg.goal_time_tolerance = rclcpp::Duration::from_seconds(2.0);

        auto send_goal_options = rclcpp_action::Client<GripperAction>::SendGoalOptions();
        auto future_goal_handle = client->async_send_goal(goal_msg, send_goal_options);
        
        if (future_goal_handle.wait_for(std::chrono::seconds(2)) != std::future_status::ready) return false;
        auto goal_handle = future_goal_handle.get();
        if (!goal_handle) return false;

        auto result_future = client->async_get_result(goal_handle);
        if (result_future.wait_for(std::chrono::seconds(5)) != std::future_status::ready) return false;

        rclcpp::sleep_for(500ms); 
        return true;
    }

    bool rightGripper(double pos) { 
        return operateGripperGeneric(gripper_right_client_, "gripper_right_finger_joint", pos); 
    }
    bool leftGripper(double pos) { 
        return operateGripperGeneric(gripper_left_client_, "gripper_left_finger_joint", pos); 
    }

    bool moveToPose(std::shared_ptr<moveit::planning_interface::MoveGroupInterface> group, 
                    const geometry_msgs::msg::Pose &pose)
    {
        group->setStartStateToCurrentState();
        group->setPoseTarget(pose);
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        if (group->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS) {
            return (group->execute(plan) == moveit::core::MoveItErrorCode::SUCCESS);
        }
        return false;
    }

    bool moveCartesian(std::shared_ptr<moveit::planning_interface::MoveGroupInterface> group, 
                       const geometry_msgs::msg::Pose &target_pose)
    {
        group->setStartStateToCurrentState();
        std::vector<geometry_msgs::msg::Pose> waypoints = {target_pose};
        moveit_msgs::msg::RobotTrajectory trajectory;
        double fraction = group->computeCartesianPath(waypoints, 0.01, 0.0, trajectory);

        if (fraction >= 0.90) {
            moveit::planning_interface::MoveGroupInterface::Plan plan;
            plan.trajectory_ = trajectory;
            return (group->execute(plan) == moveit::core::MoveItErrorCode::SUCCESS);
        }
        return false;
    }

    void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), ">>> PHASE 1: PICK <<<");

        geometry_msgs::msg::Pose target = msg->pose;
        
        target.orientation.x = 0.0; target.orientation.y = 1.0;
        target.orientation.z = 0.0; target.orientation.w = 0.0;

        rightGripper(0.00); 
        leftGripper(0.00);

        if (!moveToPose(right_arm_torso_group_, target)) return;

        geometry_msgs::msg::Pose grasp_pose = target;
        grasp_pose.position.z -= 0.15;
        if (!moveCartesian(right_arm_torso_group_, grasp_pose)) return;

        RCLCPP_INFO(this->get_logger(), ">>> GRASPING (0.40) <<<");
        rightGripper(0.40);
        rclcpp::sleep_for(std::chrono::milliseconds(500));
        attachCube("right"); 

        geometry_msgs::msg::Pose lift_pose = grasp_pose;
        lift_pose.position.z += 0.30; 
        if (!moveCartesian(right_arm_torso_group_, lift_pose)) return;
        RCLCPP_INFO(this->get_logger(), ">>> PICK DONE. STARTING HANDOVER <<<");
        
        driveBackwards();
        
        geometry_msgs::msg::Pose right_transfer = lift_pose;
        right_transfer.position.x = 0.75; 
        right_transfer.position.y = 0.00; 
        right_transfer.position.z = 1.05; 
        right_transfer.orientation = target.orientation; 

        RCLCPP_INFO(this->get_logger(), "Right Arm -> Transfer Position...");
        if (!moveToPose(right_arm_torso_group_, right_transfer)) {
             RCLCPP_WARN(this->get_logger(), "Right arm high move failed.");
        }
        
        tf2::Quaternion q_left;
        q_left.setRPY(0.0, 0.0, M_PI / 2.0);
        
        geometry_msgs::msg::Pose left_pre_grasp;
        left_pre_grasp.position.x = 0.75;
        left_pre_grasp.position.y = 0.00;
        left_pre_grasp.position.z = 1.05 - 0.40;
        left_pre_grasp.orientation = tf2::toMsg(q_left);

        RCLCPP_INFO(this->get_logger(), "Left Arm -> Move to PRE-GRASP...");
        
        if (!moveToPose(left_arm_no_torso_group_, left_pre_grasp)) {
             RCLCPP_ERROR(this->get_logger(), "Left arm pre-grasp move failed.");
             return;
        }

        RCLCPP_INFO(this->get_logger(), "Left Arm -> Cartesian Approach (Sliding UP)...");
        
        geometry_msgs::msg::Pose left_grasp_final = left_pre_grasp;
        left_grasp_final.position.z = 1.05 - 0.30; 
        
        if (!moveCartesian(left_arm_no_torso_group_, left_grasp_final)) {
            RCLCPP_WARN(this->get_logger(), "Cartesian slide failed. Trying PTP...");
            if (!moveToPose(left_arm_no_torso_group_, left_grasp_final)) {
                 RCLCPP_ERROR(this->get_logger(), "Final grasp failed.");
                 return;
            }
        }

        RCLCPP_INFO(this->get_logger(), ">>> READY FOR HANDOVER (Left gripper surrounds cube) <<<");
        leftGripper(0.30);
        rclcpp::sleep_for(std::chrono::milliseconds(500));
        detachCube("right");
        rightGripper(0.00);
        attachCube("left");
        
        RCLCPP_INFO(this->get_logger(), ">>> HANDOVER COMPLETE - Cube is now in Left Hand <<<");
 
        RCLCPP_INFO(this->get_logger(), "Right Arm -> Retreating (Up & Back)...");
        geometry_msgs::msg::Pose right_safe_lift = right_transfer;
        right_safe_lift.position.z += 0.20;
        
        right_arm_torso_group_->setPlanningTime(5.0);
        if (!moveCartesian(right_arm_torso_group_, right_safe_lift)) {
             RCLCPP_ERROR(this->get_logger(), "Right arm safety lift failed! Risk of collision.");
             moveToPose(right_arm_torso_group_, right_safe_lift);
        }
 
        driveForward();
        
        RCLCPP_INFO(this->get_logger(), ">>> PHASE: DROP <<<");
        
        geometry_msgs::msg::Pose drop_target = target;
        target.orientation.x = 0.0; target.orientation.y = 1.0;
        target.orientation.z = 0.0; target.orientation.w = 0.0;

        if (!moveToPose(left_arm_torso_group_, drop_target)) return;
        
        geometry_msgs::msg::Pose drop_final = drop_target;
        drop_final.position.z -= 0.15;
	if (!moveCartesian(left_arm_torso_group_, drop_final)) {
             moveToPose(left_arm_torso_group_, drop_final);
        }
        
        RCLCPP_INFO(this->get_logger(), ">>> RELEASING CUBE <<<");
        leftGripper(0.00);
        rclcpp::sleep_for(std::chrono::milliseconds(500));
        detachCube("left");

        geometry_msgs::msg::Pose left_escape = drop_final;
        left_escape.position.z += 0.20;
        moveCartesian(left_arm_torso_group_, left_escape);

        RCLCPP_INFO(this->get_logger(), ">>> MISSION COMPLETE <<<");

    }

    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> right_arm_torso_group_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> left_arm_no_torso_group_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> left_arm_torso_group_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> right_arm_no_torso_group_;
    
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    
    rclcpp_action::Client<GripperAction>::SharedPtr gripper_right_client_;
    rclcpp_action::Client<GripperAction>::SharedPtr gripper_left_client_;
    
    rclcpp::Client<linkattacher_msgs::srv::AttachLink>::SharedPtr attach_client_;
    rclcpp::Client<linkattacher_msgs::srv::DetachLink>::SharedPtr detach_client_;

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_;
    rclcpp::CallbackGroup::SharedPtr callback_group_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MoveToTarget>();
    node->initMoveGroups();

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}
