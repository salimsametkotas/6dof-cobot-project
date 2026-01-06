import os
import yaml
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import xacro
import sys

def load_file(package_name, file_path):
    try:
        package_path = get_package_share_directory(package_name)
        absolute_file_path = os.path.join(package_path, file_path)
    except Exception:
        print(f"\n[HATA] Paket bulunamadi: {package_name}\n")
        return None

    try:
        with open(absolute_file_path, 'r') as file:
            return file.read()
    except EnvironmentError:
        print(f"\n[HATA] Dosya Okunamadi: {absolute_file_path}\n")
        return None

def load_yaml(package_name, file_path):
    try:
        package_path = get_package_share_directory(package_name)
        absolute_file_path = os.path.join(package_path, file_path)
    except Exception:
        return None

    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError:
        print(f"\n[UYARI] YAML Dosyasi Bulunamadi (Atlandi): {absolute_file_path}\n")
        return None

def generate_launch_description():
    print("\n--- MOVEIT DEMO BASLATILIYOR ---\n")

    # 1. URDF YUKLEME
    # Burasi cok onemli: 'cobot_description' paketinde 'urdf/cobot.urdf' var mi?
    urdf_path = os.path.join(get_package_share_directory("cobot_description"), "urdf", "cobot.urdf")
    if not os.path.exists(urdf_path):
        print(f"\n[KRITIK HATA] URDF Dosyasi Yok: {urdf_path}")
        print("Lutfen 'xacro src/cobot_description/urdf/main.xacro > src/cobot_description/urdf/cobot.urdf' komutunu calistirin.\n")
        # Fallback: main.xacro deneyelim
        urdf_path = os.path.join(get_package_share_directory("cobot_description"), "urdf", "main.xacro")
    
    robot_description_config = xacro.process_file(urdf_path)
    robot_description = {"robot_description": robot_description_config.toxml()}

    # 2. SRDF YUKLEME
    robot_description_semantic_config = load_file("cobot_moveit_config", "config/main.srdf")
    # Setup Assistant bazen dosya adini robot ismiyle kaydeder (örn: cobot.srdf)
    if robot_description_semantic_config is None:
        print("[BILGI] main.srdf bulunamadi, cobot.srdf deneniyor...")
        robot_description_semantic_config = load_file("cobot_moveit_config", "config/cobot.srdf")
    
    if robot_description_semantic_config is None:
        print("[KRITIK HATA] SRDF dosyasi bulunamadi! (config klasorunu kontrol et)")
        return LaunchDescription([])

    robot_description_semantic = {"robot_description_semantic": robot_description_semantic_config}

    # 3. CONFIG DOSYALARI
    kinematics_yaml = load_yaml("cobot_moveit_config", "config/kinematics.yaml")
    joint_limits_yaml = load_yaml("cobot_moveit_config", "config/joint_limits.yaml")
    
    # OMPL (Eksikse varsayilan bos dict donelim, cokmesi engellensin)
    ompl_planning_yaml = load_yaml("cobot_moveit_config", "config/ompl_planning.yaml")
    if ompl_planning_yaml is None:
        ompl_planning_yaml = {}

    ompl_planning_pipeline_config = {
        "move_group": {
            "planning_plugin": "ompl_interface/OMPLPlanner",
            "request_adapters": """default_planner_request_adapters/AddTimeOptimalParameterization default_planner_request_adapters/ResolveConstraintFrames default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints""",
            "start_state_max_bounds_error": 0.1,
        }
    }
    ompl_planning_pipeline_config["move_group"].update(ompl_planning_yaml)

    # 4. NODE'LAR
    run_move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics_yaml,
            joint_limits_yaml,
            ompl_planning_pipeline_config,
            {"use_sim_time": False},
        ],
    )

    rviz_config_file = os.path.join(get_package_share_directory("cobot_moveit_config"), "config", "moveit.rviz")
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics_yaml,
            joint_limits_yaml,
        ],
    )

    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["0", "0", "0", "0", "0", "0", "world", "base_link"],
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    # ROS2 Controllers (Eksikse hata vermesin, gecsin)
    ros2_controllers_path = os.path.join(get_package_share_directory("cobot_moveit_config"), "config", "ros2_controllers.yaml")
    if not os.path.exists(ros2_controllers_path):
        print(f"[UYARI] ros2_controllers.yaml bulunamadi: {ros2_controllers_path}")
        # Bos bir liste ile devam et, Fake Controller calismaz ama RViz acilir
        ros2_control_node = Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=[robot_description],
            output="screen",
        )
    else:
        ros2_control_node = Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=[robot_description, ros2_controllers_path],
            output="screen",
        )

    return LaunchDescription([
        static_tf,
        robot_state_publisher,
        run_move_group_node,
        rviz_node,
        ros2_control_node,
    ])
