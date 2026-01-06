#ifndef COBOT_CONTROLLER__COBOT_CONTROLLER_HPP_
#define COBOT_CONTROLLER__COBOT_CONTROLLER_HPP_

#include "controller_interface/controller_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

#include "geometry_msgs/msg/vector3.hpp"

#include "rclcpp/node_interfaces/node_parameters_interface.hpp"
#include "cobot_controller/cobot_controller_parameters.hpp"

#include "cobot_interfaces/action/forward_kinematic.hpp"
#include "cobot_interfaces/action/inverse_kinematic.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include <cmath>

namespace cobot_controller
{
    class CobotController : public controller_interface::ControllerInterface
    {

    public:
        using ForwardKinematic = cobot_interfaces::action::ForwardKinematic;
        using GoalHandleForwardKinematic = rclcpp_action::ServerGoalHandle<ForwardKinematic>;

        using InverseKinematic = cobot_interfaces::action::InverseKinematic;
        using GoalHandleInverseKinematic = rclcpp_action::ServerGoalHandle<InverseKinematic>;

        controller_interface::CallbackReturn on_init() override;
        controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &previous_state) override;
        controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;
        controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override;
        controller_interface::return_type update(const rclcpp::Time &time, const rclcpp::Duration &period) override;
        controller_interface::InterfaceConfiguration command_interface_configuration() const override;
        controller_interface::InterfaceConfiguration state_interface_configuration() const override;

    private:
        float angle_;

        double part_1_joint_state_;
        double part_2_joint_state_;
        double part_3_joint_state_;
        double part_4_joint_state_;
        double part_5_joint_state_;
        double part_6_joint_state_;
        double finger_1_joint_state_;
        double finger_2_joint_state_;

        std::array<std::string, 6> joint_names_;
        std::array<std::string, 2> gripper_joint_names_;

        std::array<double, 6> goals_;

        rclcpp_action::Server<ForwardKinematic>::SharedPtr forward_kinematic_action_server_;
        rclcpp_action::GoalResponse forward_handle_goal(
            const rclcpp_action::GoalUUID &uuid,
            std::shared_ptr<const ForwardKinematic::Goal> goal);
        rclcpp_action::CancelResponse forward_handle_cancel(
            const std::shared_ptr<GoalHandleForwardKinematic> goal_handle);
        void forward_handle_accepted(const std::shared_ptr<GoalHandleForwardKinematic> goal_handle);
        void forward_execute(const std::shared_ptr<GoalHandleForwardKinematic> goal_handle);

        rclcpp_action::Server<InverseKinematic>::SharedPtr inverse_kinematic_action_server_;
        rclcpp_action::GoalResponse inverse_handle_goal(
            const rclcpp_action::GoalUUID &uuid,
            std::shared_ptr<const InverseKinematic::Goal> goal);
        rclcpp_action::CancelResponse inverse_handle_cancel(
            const std::shared_ptr<GoalHandleInverseKinematic> goal_handle);
        void inverse_handle_accepted(const std::shared_ptr<GoalHandleInverseKinematic> goal_handle);
        void inverse_execute(const std::shared_ptr<GoalHandleInverseKinematic> goal_handle);

        std::array<double, 3> forward_kinematic(double Q1, double Q2, double Q3, double Q4);
        std::array<double, 6> inverse_kinematic(double x, double y, double z);

    protected:
        std::shared_ptr<ParamListener> param_listener_;
        Params params_;
    };
}

#endif