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
    
    # 1. URDF Yükle (Robot Tanımı)
    urdf_path = os.path.join(get_package_share_directory("cobot_description"), "urdf", "cobot.urdf")
    # Eğer cobot.urdf yoksa main.xacro dene
    if not os.path.exists(urdf_path):
         urdf_path = os.path.join(get_package_share_directory("cobot_description"), "urdf", "main.xacro")
    
    robot_description_config = xacro.process_file(urdf_path)
    robot_description = {"robot_description": robot_description_config.toxml()}

    # 2. SRDF Yükle (MoveIt Grupları)
    robot_description_semantic_config = load_file("cobot_moveit_config", "config/main.srdf")
    # Dosya adı farklı olabilir kontrolü
    if robot_description_semantic_config is None:
        robot_description_semantic_config = load_file("cobot_moveit_config", "config/cobot.srdf")
        
    robot_description_semantic = {"robot_description_semantic": robot_description_semantic_config}

    # 3. Kinematik Ayarları
    kinematics_yaml = load_yaml("cobot_moveit_config", "config/kinematics.yaml")

    # 4. RViz Konfigürasyon Dosyası
    rviz_config_file = os.path.join(get_package_share_directory("cobot_moveit_config"), "config", "moveit.rviz")

    # --- RViz Node ---
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
        ],
    )

    return LaunchDescription([
        rviz_node,
    ])
