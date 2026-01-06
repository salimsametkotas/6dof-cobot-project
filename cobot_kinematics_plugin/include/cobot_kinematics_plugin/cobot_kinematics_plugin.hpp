#ifndef COBOT_KINEMATICS_PLUGIN_HPP
#define COBOT_KINEMATICS_PLUGIN_HPP

#include <moveit/kinematics_base/kinematics_base.h>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>

namespace cobot_kinematics_plugin
{
class CobotKinematicsPlugin : public kinematics::KinematicsBase
{
public:
 
  virtual bool initialize(const rclcpp::Node::SharedPtr& node, const moveit::core::RobotModel& robot_model,
                          const std::string& group_name, const std::string& base_frame,
                          const std::vector<std::string>& tip_frames, double search_discretization) override;

  
  virtual bool getPositionIK(const geometry_msgs::msg::Pose& ik_pose,
                             const std::vector<double>& ik_seed_state,
                             std::vector<double>& solution,
                             moveit_msgs::msg::MoveItErrorCodes& error_code,
                             const kinematics::KinematicsQueryOptions& options = kinematics::KinematicsQueryOptions()) const override;

  virtual bool getPositionFK(const std::vector<std::string>& link_names,
                             const std::vector<double>& joint_angles,
                             std::vector<geometry_msgs::msg::Pose>& poses) const override;

  
  virtual bool searchPositionIK(const geometry_msgs::msg::Pose& ik_pose,
                                const std::vector<double>& ik_seed_state,
                                double timeout,
                                std::vector<double>& solution,
                                moveit_msgs::msg::MoveItErrorCodes& error_code,
                                const kinematics::KinematicsQueryOptions& options = kinematics::KinematicsQueryOptions()) const override;

  virtual bool searchPositionIK(const geometry_msgs::msg::Pose& ik_pose,
                                const std::vector<double>& ik_seed_state,
                                double timeout,
                                const std::vector<double>& consistency_limits,
                                std::vector<double>& solution,
                                moveit_msgs::msg::MoveItErrorCodes& error_code,
                                const kinematics::KinematicsQueryOptions& options = kinematics::KinematicsQueryOptions()) const override;

  virtual bool searchPositionIK(const geometry_msgs::msg::Pose& ik_pose,
                                const std::vector<double>& ik_seed_state,
                                double timeout,
                                std::vector<double>& solution,
                                const IKCallbackFn& solution_callback,
                                moveit_msgs::msg::MoveItErrorCodes& error_code,
                                const kinematics::KinematicsQueryOptions& options = kinematics::KinematicsQueryOptions()) const override;

  virtual bool searchPositionIK(const geometry_msgs::msg::Pose& ik_pose,
                                const std::vector<double>& ik_seed_state,
                                double timeout,
                                const std::vector<double>& consistency_limits,
                                std::vector<double>& solution,
                                const IKCallbackFn& solution_callback,
                                moveit_msgs::msg::MoveItErrorCodes& error_code,
                                const kinematics::KinematicsQueryOptions& options = kinematics::KinematicsQueryOptions()) const override;

  virtual const std::vector<std::string>& getJointNames() const override;
  virtual const std::vector<std::string>& getLinkNames() const override;

private:
 
  std::vector<double> calculate_analytical_ik(double x, double y, double z) const;

  std::vector<std::string> joint_names_;
  std::vector<std::string> link_names_;
  rclcpp::Node::SharedPtr node_;
};
}  

#endif
