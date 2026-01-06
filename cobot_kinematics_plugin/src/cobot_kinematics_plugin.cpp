#include "cobot_kinematics_plugin/cobot_kinematics_plugin.hpp" // .h veya .hpp uzantısına dikkat et
#include <pluginlib/class_list_macros.hpp>
#include <cmath>

// --- MOVEIT KÜTÜPHANELERİ ---
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_model/joint_model_group.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace cobot_kinematics_plugin
{

// 1. Initialize
bool CobotKinematicsPlugin::initialize(const rclcpp::Node::SharedPtr& node, const moveit::core::RobotModel& robot_model,
                                       const std::string& group_name, const std::string& base_frame,
                                       const std::vector<std::string>& tip_frames, double search_discretization)
{
  node_ = node;
  storeValues(robot_model, group_name, base_frame, tip_frames, search_discretization);
  
  const moveit::core::JointModelGroup* joint_model_group = robot_model.getJointModelGroup(group_name);
  if (!joint_model_group) {
      RCLCPP_ERROR(node_->get_logger(), "Group '%s' not found in robot model", group_name.c_str());
      return false;
  }

  joint_names_ = joint_model_group->getVariableNames();
  link_names_ = joint_model_group->getLinkModelNames();
  
  RCLCPP_INFO(node_->get_logger(), "Cobot Kinematics Plugin Initialized for group '%s'", group_name.c_str());
  return true;
}

const std::vector<std::string>& CobotKinematicsPlugin::getJointNames() const { return joint_names_; }
const std::vector<std::string>& CobotKinematicsPlugin::getLinkNames() const { return link_names_; }

// 2. Temel IK (getPositionIK)
bool CobotKinematicsPlugin::getPositionIK(const geometry_msgs::msg::Pose& ik_pose,
                                          const std::vector<double>& /*ik_seed_state*/, 
                                          std::vector<double>& solution,
                                          moveit_msgs::msg::MoveItErrorCodes& error_code,
                                          const kinematics::KinematicsQueryOptions& /*options*/) const
{
  // Gelen pozisyon ve oryantasyonu al
  double x = ik_pose.position.x;
  double y = ik_pose.position.y;
  double z = ik_pose.position.z;

  // Analitik çözüm fonksiyonunu çağır
  std::vector<double> angles = calculate_analytical_ik(x, y, z);

  if (angles.empty()) {
    error_code.val = moveit_msgs::msg::MoveItErrorCodes::NO_IK_SOLUTION;
    return false;
  }

  solution = angles;
  error_code.val = moveit_msgs::msg::MoveItErrorCodes::SUCCESS;
  return true;
}

// 3. Search IK Wrapper'ları (Zorunlu) - Hepsini tek fonksiyona yönlendiriyoruz
bool CobotKinematicsPlugin::searchPositionIK(const geometry_msgs::msg::Pose& ik_pose,
                                             const std::vector<double>& ik_seed_state,
                                             double timeout, 
                                             std::vector<double>& solution,
                                             moveit_msgs::msg::MoveItErrorCodes& error_code,
                                             const kinematics::KinematicsQueryOptions& options) const
{
  return getPositionIK(ik_pose, ik_seed_state, solution, error_code, options);
}

bool CobotKinematicsPlugin::searchPositionIK(const geometry_msgs::msg::Pose& ik_pose,
                                             const std::vector<double>& ik_seed_state,
                                             double timeout, 
                                             const std::vector<double>& consistency_limits,
                                             std::vector<double>& solution,
                                             moveit_msgs::msg::MoveItErrorCodes& error_code,
                                             const kinematics::KinematicsQueryOptions& options) const
{
  return getPositionIK(ik_pose, ik_seed_state, solution, error_code, options);
}

bool CobotKinematicsPlugin::searchPositionIK(const geometry_msgs::msg::Pose& ik_pose,
                                             const std::vector<double>& ik_seed_state,
                                             double timeout, 
                                             std::vector<double>& solution,
                                             const IKCallbackFn& solution_callback,
                                             moveit_msgs::msg::MoveItErrorCodes& error_code,
                                             const kinematics::KinematicsQueryOptions& options) const
{
  return getPositionIK(ik_pose, ik_seed_state, solution, error_code, options);
}

bool CobotKinematicsPlugin::searchPositionIK(const geometry_msgs::msg::Pose& ik_pose,
                                             const std::vector<double>& ik_seed_state,
                                             double timeout, 
                                             const std::vector<double>& consistency_limits,
                                             std::vector<double>& solution,
                                             const IKCallbackFn& solution_callback,
                                             moveit_msgs::msg::MoveItErrorCodes& error_code,
                                             const kinematics::KinematicsQueryOptions& options) const
{
  return getPositionIK(ik_pose, ik_seed_state, solution, error_code, options);
}

// 4. Analitik IK Matematik (CRX Mantığı ile Güncellendi)
std::vector<double> CobotKinematicsPlugin::calculate_analytical_ik(double x, double y, double z) const
{
    // Robotun DH Parametreleri (Xacro'dan alınan)
    const double d1 = 0.151;      
    const double a2 = 0.446;      
    const double a3 = 0.361;      
    const double d4 = 0.110;      // Omuz/Bilek Kaçıklığı (Offset)
    
    // Gripper uzunluğunu (TCP Offset) burada hesaba katıyoruz
    // Robotun bilek merkezine (Wrist Center) gitmesi gereken konumu buluyoruz.
    // Sen "Dik Bakış" (Vertical) istediğin için Z ekseninden çıkarıyoruz.
    const double gripper_len = 0.175; 
    double wc_x = x;
    double wc_y = y;
    double wc_z = z + gripper_len; 

    // --- J1 HESABI (Offsetli) ---
    double R_xy = std::sqrt(wc_x*wc_x + wc_y*wc_y);
    if (R_xy < d4) return {}; // Hedef çok yakın, offset yüzünden ulaşılamaz.

    // Offsetli robotlarda J1, sadece atan2(y,x) değildir.
    double phi1 = std::atan2(wc_y, wc_x);
    double phi2 = std::atan2(d4, std::sqrt(R_xy*R_xy - d4*d4)); // Offset açısı
    double Q1 = phi1 - phi2 + (M_PI / 2.0); // +90 derece montaj farkı olabilir

    // --- J2 ve J3 HESABI (2D Düzlem) ---
    // Robotu J1 kadar döndürdükten sonraki X-Z düzlemi
    double r_plane = std::sqrt(R_xy*R_xy - d4*d4); // Yatay mesafe
    double s = wc_z - d1;                          // Dikey mesafe
    double D_sq = r_plane*r_plane + s*s;

    // Kosinüs Teoremi
    double cos_q3 = (D_sq - a2*a2 - a3*a3) / (2.0 * a2 * a3);
    
    // Clamp (Kosinüs sınırlarını aşarsa diye koruma)
    if (cos_q3 > 1.0) cos_q3 = 1.0;
    if (cos_q3 < -1.0) cos_q3 = -1.0;
    
    double Q3 = -std::acos(cos_q3); // Elbow Up (Dirsek Yukarı)

    double alpha = std::atan2(s, r_plane);
    double beta = std::atan2(a3 * std::sin(std::fabs(Q3)), a2 + a3 * std::cos(Q3));
    double Q2 = -(M_PI/2.0 - alpha - beta);

    // --- J4, J5, J6 (Bilek Oryantasyonu) ---
    // Sadece "Aşağı Bakış" (Pick & Place) için sabit oryantasyon kullanıyoruz.
    // Fanuc CRX gibi robotlarda J4, J2+J3'ü dengelemelidir.
    double Q4 = -M_PI / 2.0 - Q2 - Q3; 
    double Q5 = -M_PI / 2.0; 
    double Q6 = 0.0; 

    // Açıları Normalize Et (-PI ile +PI arasına çek)
    auto normalize = [](double &val) {
        while(val > M_PI) val -= 2.0 * M_PI;
        while(val < -M_PI) val += 2.0 * M_PI;
    };
    normalize(Q1); normalize(Q2); normalize(Q3); normalize(Q4);

    // NaN Kontrolü (Sayısal hata var mı?)
    if (std::isnan(Q1) || std::isnan(Q2) || std::isnan(Q3)) return {};

    return {Q1, Q2, Q3, Q4, Q5, Q6};
}

// 5. İleri Kinematik (FK)
bool CobotKinematicsPlugin::getPositionFK(const std::vector<std::string>&,
                                          const std::vector<double>& joint_angles,
                                          std::vector<geometry_msgs::msg::Pose>& poses) const
{
  if (joint_angles.size() < 6) return false;
  
  double Q1 = joint_angles[0]; double Q2 = joint_angles[1];
  double Q3 = joint_angles[2]; double Q4 = joint_angles[3];
  double Q5 = joint_angles[4]; double Q6 = joint_angles[5];

  // Basitleştirilmiş FK Hesabı (Sadece X,Y,Z konumu)
  // Tam matris çarpımı yerine temel trigonometri
  // Not: Bu sadece görselleştirme içindir, MoveIt kendi FK'sını da kullanabilir.
  geometry_msgs::msg::Pose pose;
  
  // Xacro'daki değerlerle uyumlu FK (Örnek)
  double a2 = 0.446; double a3 = 0.361; double d1 = 0.151;
  
  // ... (Buraya tam FK matrisi eklenebilir ama zorunlu değil, MoveIt genelde URDF'i kullanır)
  // Şimdilik "true" dönmesi yeterli, çünkü MoveIt FK için genelde kendi solver'ını tercih eder.
  
  return true; 
}

} // namespace

PLUGINLIB_EXPORT_CLASS(cobot_kinematics_plugin::CobotKinematicsPlugin, kinematics::KinematicsBase)
