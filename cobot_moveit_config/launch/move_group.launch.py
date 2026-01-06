import os
import yaml
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import xacro

def load_file(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, 'r') as file:
            return file.read()
    except EnvironmentError:
        return None

def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError:
        return None

def generate_launch_description():
    
    # 1. URDF (Robot Tanımı)
    urdf_path = os.path.join(get_package_share_directory("cobot_description"), "urdf", "cobot.urdf")
    if not os.path.exists(urdf_path):
         urdf_path = os.path.join(get_package_share_directory("cobot_description"), "urdf", "main.xacro")

    robot_description_config = xacro.process_file(urdf_path)
    robot_description = {"robot_description": robot_description_config.toxml()}

    # 2. SRDF (MoveIt Grupları)
    robot_description_semantic_config = load_file("cobot_moveit_config", "config/main.srdf")
    if robot_description_semantic_config is None:
        robot_description_semantic_config = load_file("cobot_moveit_config", "config/cobot.srdf")
        
    robot_description_semantic = {"robot_description_semantic": robot_description_semantic_config}

    # 3. Kinematik (Senin Plugin)
    kinematics_yaml = load_yaml("cobot_moveit_config", "config/kinematics.yaml")

    # 4. Limitler
    joint_limits_yaml = load_yaml("cobot_moveit_config", "config/joint_limits.yaml")

    # 5. OMPL (Planlama)
    ompl_planning_pipeline_config = {
        "move_group": {
            "planning_plugin": "ompl_interface/OMPLPlanner",
            "request_adapters": """default_planner_request_adapters/AddTimeOptimalParameterization default_planner_request_adapters/ResolveConstraintFrames default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints""",
            "start_state_max_bounds_error": 0.1,
        }
    }
    ompl_planning_yaml = load_yaml("cobot_moveit_config", "config/ompl_planning.yaml")
    if ompl_planning_yaml:
        ompl_planning_pipeline_config["move_group"].update(ompl_planning_yaml)

    # 6. Trajectory Execution
    trajectory_execution = {
        "moveit_manage_controllers": True,
        "trajectory_execution.allowed_execution_duration_scaling": 1.2,
        "trajectory_execution.allowed_goal_duration_margin": 0.5,
        "trajectory_execution.allowed_start_tolerance": 0.01,
    }
    
    # YENİ: moveit_controllers.yaml dosyasını yüklüyoruz
    moveit_controllers = load_yaml("cobot_moveit_config", "config/moveit_controllers.yaml")

    # 7. Move Group Node
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
            trajectory_execution,
            moveit_controllers,
            {"use_sim_time": True}, 
        ],
    )

    # 8. Static TF (EKSİK OLAN PARÇA BU)
    # Robotun 'base_link'ini 'world'e bağlar.
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["0", "0", "0", "0", "0", "0", "world", "base_link"],
    )

    return LaunchDescription([
        static_tf, # Bunu ekledik
        run_move_group_node,
    ])
