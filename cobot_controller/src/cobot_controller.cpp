#include "cobot_controller/cobot_controller.hpp"

namespace cobot_controller
{
    constexpr auto DEFAULT_GOAL_TOPIC = "~/goal";

    using hardware_interface::HW_IF_POSITION;

    controller_interface::CallbackReturn CobotController::on_init()
    {
        RCLCPP_INFO(get_node()->get_logger(), "on_init() fonksiyonu basladi.");

        try
        {
            param_listener_ = std::make_shared<ParamListener>(get_node());
            params_ = param_listener_->get_params();
        }
        catch (const std::exception &e)
        {
            fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
            return controller_interface::CallbackReturn::ERROR;
        }

        joint_names_ = {
            params_.part_1_joint,
            params_.part_2_joint,
            params_.part_3_joint,
            params_.part_4_joint,
            params_.part_5_joint,
            params_.part_6_joint,
        };

        gripper_joint_names_ = {
            params_.finger_1_joint,
            params_.finger_2_joint,
        };

        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::InterfaceConfiguration CobotController::command_interface_configuration() const
    {
        RCLCPP_INFO(get_node()->get_logger(), "Komut arayüzü konfigüre edildi.");
        std::vector<std::string> conf_names;

        for (const auto &joint_name_ : joint_names_)
        {
            conf_names.push_back(joint_name_ + "/" + HW_IF_POSITION);
        }

        for (const auto &joint_name_ : gripper_joint_names_)
        {
            conf_names.push_back(joint_name_ + "/" + HW_IF_POSITION);
        }

        return {controller_interface::interface_configuration_type::INDIVIDUAL, conf_names};
    }

    controller_interface::InterfaceConfiguration CobotController::state_interface_configuration() const
    {
        RCLCPP_INFO(get_node()->get_logger(), "Durum arayüzü konfigüre edildi.");

        std::vector<std::string> conf_names;

        for (const auto &joint_name_ : joint_names_)
        {
            conf_names.push_back(joint_name_ + "/" + HW_IF_POSITION);
        }

        for (const auto &joint_name_ : gripper_joint_names_)
        {
            conf_names.push_back(joint_name_ + "/" + HW_IF_POSITION);
        }

        return {controller_interface::interface_configuration_type::INDIVIDUAL, conf_names};
    }

    controller_interface::CallbackReturn CobotController::on_configure(const rclcpp_lifecycle::State &)
    {
        forward_kinematic_action_server_ = rclcpp_action::create_server<ForwardKinematic>(
            get_node(),
            "~/forward/goal",
            std::bind(&CobotController::forward_handle_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&CobotController::forward_handle_cancel, this, std::placeholders::_1),
            std::bind(&CobotController::forward_handle_accepted, this, std::placeholders::_1),
            rcl_action_server_get_default_options(),
            nullptr);

        inverse_kinematic_action_server_ = rclcpp_action::create_server<InverseKinematic>(
            get_node(),
            "~/inverse/goal",
            std::bind(&CobotController::inverse_handle_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&CobotController::inverse_handle_cancel, this, std::placeholders::_1),
            std::bind(&CobotController::inverse_handle_accepted, this, std::placeholders::_1),
            rcl_action_server_get_default_options(),
            nullptr);

        return CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn CobotController::on_activate(const rclcpp_lifecycle::State &)
    {
        angle_ = params_.angle_degree * M_PI / 180;
        return CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn CobotController::on_deactivate(const rclcpp_lifecycle::State &)
    {
        angle_ = 0;

        return CallbackReturn::SUCCESS;
    }

    controller_interface::return_type CobotController::update(const rclcpp::Time &time, const rclcpp::Duration &period)
    {
        // double dt = period.seconds();
        (void)time;
        (void)period;

        for (size_t i = 0; i < 6; i++)
        {
            double state = state_interfaces_[i].get_value();
            double command = state;
            double goal = goals_[i];

            if (fabs(state - goal) > angle_)
            {
                command = state - (angle_ * copysign(1, state - goal));
            }

            command_interfaces_[i].set_value(command);
        }

        return controller_interface::return_type::OK;
    }

    rclcpp_action::GoalResponse CobotController::forward_handle_goal(
        const rclcpp_action::GoalUUID &uuid,
        std::shared_ptr<const ForwardKinematic::Goal> goal)
    {
        if (goal->goal.size() != goals_.size())
        {
            RCLCPP_ERROR(get_node()->get_logger(), "Gelen Goal dizisi boyutu (%zu) kontrolcü eklem sayısına (%zu) uymuyor!",
                         goal->goal.size(), goals_.size());
            return rclcpp_action::GoalResponse::REJECT;
        }

        std::copy(
            goal->goal.begin(),
            goal->goal.end(),
            goals_.begin());

        std::string s;
        for (double v : goals_)
        {
            s += std::to_string(v) + " ";
        }

        RCLCPP_INFO(get_node()->get_logger(), "Goal array: %s", s.c_str());
        (void)uuid;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse CobotController::forward_handle_cancel(const std::shared_ptr<GoalHandleForwardKinematic> goal_handle)
    {
        RCLCPP_INFO(get_node()->get_logger(), "Received request to cancel goal");
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void CobotController::forward_handle_accepted(const std::shared_ptr<GoalHandleForwardKinematic> goal_handle)
    {
        std::thread{std::bind(&CobotController::forward_execute, this, std::placeholders::_1), goal_handle}.detach();
    }

    void CobotController::forward_execute(const std::shared_ptr<GoalHandleForwardKinematic> goal_handle)
    {
        RCLCPP_INFO(get_node()->get_logger(), "Executing goal");
        rclcpp::Rate loop_rate(10);

        const auto goal = goal_handle->get_goal();
        auto feedback = std::make_shared<ForwardKinematic::Feedback>();
        auto result = std::make_shared<ForwardKinematic::Result>();

        while (rclcpp::ok())
        {
            if (goal_handle->is_canceling())
            {
                goal_handle->canceled(result);
                RCLCPP_WARN(get_node()->get_logger(), "Goal iptal edildi.");
                return;
            }

            bool all_joints_reached = true;

            for (size_t i = 0; i < goals_.size(); i++)
            {
                double current_state = state_interfaces_[i].get_value();
                double target_goal = goals_[i];

                std::array<double, 3> feedback_forward = forward_kinematic(state_interfaces_[0].get_value(), state_interfaces_[1].get_value(), state_interfaces_[2].get_value(), state_interfaces_[3].get_value());

                feedback->x = feedback_forward[0];
                feedback->y = feedback_forward[1];
                feedback->z = feedback_forward[2];

                goal_handle->publish_feedback(feedback);
                if (fabs(current_state - target_goal) > angle_)
                {
                    all_joints_reached = false;
                    break;
                }
            }

            if (all_joints_reached)
            {
                std::array<double, 3> feedback_forward = forward_kinematic(state_interfaces_[0].get_value(), state_interfaces_[1].get_value(), state_interfaces_[2].get_value(), state_interfaces_[3].get_value());
                result->x = feedback_forward[0];
                result->y = feedback_forward[1];
                result->z = feedback_forward[2];

                goal_handle->succeed(result);
                RCLCPP_INFO(get_node()->get_logger(), "Goal başarıyla tamamlandı (Hedefe ulaşıldı).");
                return;
            }

            loop_rate.sleep();
        }

        if (!rclcpp::ok())
        {
            goal_handle->abort(result);
            RCLCPP_ERROR(get_node()->get_logger(), "Sistem kapandığı için Goal iptal edildi (Abort).");
        }
    }

    std::array<double, 3> CobotController::forward_kinematic(double Q1, double Q2, double Q3, double Q4)
    {
        double x, y, z;
        x = 0.13 * std::sin(Q1) - std::cos(Q1) * (0.446 * std::sin(Q2) + 0.361 * std::sin(Q2 + Q3) + 0.1425 * std::sin(Q2 + Q3 + Q4));
        y = -0.13 * std::cos(Q1) - std::sin(Q1) * (0.446 * std::sin(Q2) + 0.361 * std::sin(Q2 + Q3) + 0.1425 * std::sin(Q2 + Q3 + Q4));
        z = 0.446 * std::cos(Q2) + 0.361 * std::cos(Q2 + Q3) + 0.1425 * std::cos(Q2 + Q3 + Q4);
        return {x, y, z};
    }

    rclcpp_action::GoalResponse CobotController::inverse_handle_goal(
        const rclcpp_action::GoalUUID &uuid,
        std::shared_ptr<const InverseKinematic::Goal> goal)
    {
        RCLCPP_INFO(get_node()->get_logger(), "Hedef Geldi");
        std::array<double, 6> dummy = inverse_kinematic(goal->x, goal->y, goal->z);

        std::copy(
            dummy.begin(),
            dummy.end(),
            goals_.begin());

        std::string s;
        for (double v : goals_)
        {
            s += std::to_string(v) + " ";
        }

        RCLCPP_INFO(get_node()->get_logger(), "Goal array: %s", s.c_str());

        (void)uuid;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse CobotController::inverse_handle_cancel(const std::shared_ptr<GoalHandleInverseKinematic> goal_handle)
    {
        RCLCPP_INFO(get_node()->get_logger(), "Received request to cancel goal");
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void CobotController::inverse_handle_accepted(const std::shared_ptr<GoalHandleInverseKinematic> goal_handle)
    {
        std::thread{std::bind(&CobotController::inverse_execute, this, std::placeholders::_1), goal_handle}.detach();
    }

    void CobotController::inverse_execute(const std::shared_ptr<GoalHandleInverseKinematic> goal_handle)
    {
        RCLCPP_INFO(get_node()->get_logger(), "Executing goal");
        rclcpp::Rate loop_rate(10);

        const auto goal = goal_handle->get_goal();
        auto feedback = std::make_shared<InverseKinematic::Feedback>();
        auto result = std::make_shared<InverseKinematic::Result>();

        while (rclcpp::ok())
        {
            if (goal_handle->is_canceling())
            {
                goal_handle->canceled(result);
                RCLCPP_WARN(get_node()->get_logger(), "Goal iptal edildi.");
                return;
            }

            bool all_joints_reached = true;

            // 1. Hedefe varıldı mı kontrolü
            for (size_t i = 0; i < goals_.size(); i++)
            {
                double current_state = state_interfaces_[i].get_value();
                double target_goal = goals_[i];

                if (fabs(current_state - target_goal) > angle_)
                {
                    all_joints_reached = false;
                }
            }

            
            
            // A) Anlık Konum (X, Y, Z) Hesabı (FK Formülü ile)
            std::array<double, 3> xyz = forward_kinematic(
                state_interfaces_[0].get_value(), 
                state_interfaces_[1].get_value(), 
                state_interfaces_[2].get_value(), 
                state_interfaces_[3].get_value()
            );
            
            feedback->current_x = xyz[0];
            feedback->current_y = xyz[1];
            feedback->current_z = xyz[2];

            // B) Anlık Açılar (J1, J2, J3...)
            feedback->current_joints.clear(); // Listeyi temizle
            for(size_t i=0; i<6; i++) {
                // Anlık motor açısını oku ve feedback listesine ekle
                feedback->current_joints.push_back(state_interfaces_[i].get_value());
            }

            // Veriyi Arayüze Gönder
            goal_handle->publish_feedback(feedback);
            // ----------------------------------------

            if (all_joints_reached)
            {
                // Hedefe varınca son durumu 'result' olarak döndür
                result->joint_angles.assign(goals_.begin(), goals_.end());
                
                goal_handle->succeed(result);
                RCLCPP_INFO(get_node()->get_logger(), "Goal başarıyla tamamlandı (Hedefe ulaşıldı).");
                return;
            }

            loop_rate.sleep();
        }

        if (!rclcpp::ok())
        {
            goal_handle->abort(result);
            RCLCPP_ERROR(get_node()->get_logger(), "Sistem kapandığı için Goal iptal edildi (Abort).");
        }
    }

     std::array<double, 6> CobotController::inverse_kinematic(double x, double y, double z)
    {
        double Q1, Q2, Q3, Q4, Q5, Q6;

        Q1 = state_interfaces_[0].get_value();
        Q2 = state_interfaces_[1].get_value();
        Q3 = state_interfaces_[2].get_value();
        Q4 = state_interfaces_[3].get_value();
        Q5 = state_interfaces_[4].get_value();
        Q6 = state_interfaces_[5].get_value();

        double r11 = -std::sin(Q1) * std::sin(Q5) * std::cos(Q6) + std::cos(Q1) * (-std::sin(Q2 + Q3 + Q4) * std::sin(Q6) + std::cos(Q2 + Q3 + Q4) * std::cos(Q5) * std::cos(Q6));
        double r12 = std::sin(Q1) * std::sin(Q5) * std::sin(Q6) - std::cos(Q1) * (std::sin(Q2 + Q3 + Q4) * std::cos(Q6) + std::cos(Q2 + Q3 + Q4) * std::cos(Q5) * std::sin(Q6));
        double r13 = std::sin(Q1) * std::cos(Q5) + std::cos(Q1) * std::cos(Q2 + Q3 + Q4) * std::sin(Q5);

        double r21 = std::cos(Q1) * std::sin(Q5) * std::cos(Q6) + std::sin(Q1) * (-std::sin(Q2 + Q3 + Q4) * std::sin(Q6) + std::cos(Q2 + Q3 + Q4) * std::cos(Q5) * std::cos(Q6));
        double r22 = -std::cos(Q1) * std::sin(Q5) * std::sin(Q6) - std::sin(Q1) * (std::sin(Q2 + Q3 + Q4) * std::cos(Q6) + std::cos(Q2 + Q3 + Q4) * std::cos(Q5) * std::sin(Q6));
        double r23 = -std::cos(Q1) * std::cos(Q5) + std::sin(Q1) * std::cos(Q2 + Q3 + Q4) * std::sin(Q5);

        double r31 = std::cos(Q2 + Q3 + Q4) * std::sin(Q6) + std::sin(Q2 + Q3 + Q4) * std::cos(Q5) * std::cos(Q6);
        double r32 = std::cos(Q2 + Q3 + Q4) * std::cos(Q6) - std::sin(Q2 + Q3 + Q4) * std::cos(Q5) * std::sin(Q6);
        

        double c = -x * std::cos(Q1) - y * std::sin(Q1) - 0.1425 * ((r31 * std::cos(Q6) - r32 * std::sin(Q6)) / std::cos(Q5));
        double d = z - 0.1425 * (r32 * std::cos(Q6) + r31 * std::sin(Q6));
        double A = -2 * 0.446 * c;
        double B = -2 * 0.446 * d;
        double E = std::pow(0.446, 2) + std::pow(c, 2) + std::pow(d, 2) + std::pow(0.361, 2);

        double Q1_ = 2 * std::atan((x + std::sqrt(std::pow(y, 2) + std::pow(x, 2) - 0.13)) / (0.13 - y));
        double Q6_ = std::atan2(r12 * std::sin(Q1) - r22 * std::cos(Q1), r21 * std::cos(Q1) - r11 * std::sin(Q1));
        double Q5_ = std::atan2((r21 * std::cos(Q1) - r11 * std::sin(Q1)) * std::cos(Q6) + std::sin(Q6) * (r12 * std::sin(Q1) - r22 * std::cos(Q1)), r13 * std::sin(Q1) - r23 * std::cos(Q1));
        double Q2_ = 2 * std::atan((-A - std::sqrt(std::pow(B, 2) + std::pow(A, 2) - std::pow(E, 2))) / (E - B));
        double Q3_ = std::atan2(c - 0.446 * std::sin(Q2), d - 0.446 * std::cos(Q2)) - Q2;
        double Q4_ = std::atan2((r31 * std::cos(Q6) - r32 * std::sin(Q6)) / std::cos(Q5), r32 * std::cos(Q6) + r31 * std::sin(Q6)) - Q2 - Q3;

        return {Q1_, Q2_, Q3_, Q4_, Q5_, Q6_};
    }
}
#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(cobot_controller::CobotController, controller_interface::ControllerInterface)
