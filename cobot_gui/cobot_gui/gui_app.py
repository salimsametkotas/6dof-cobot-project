#!/usr/bin/env python3
import sys
import os
import datetime
import cv2
import numpy as np
import threading
import time
import math

from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, 
                             QHBoxLayout, QLabel, QComboBox, QLineEdit, 
                             QPushButton, QTabWidget, QFormLayout, QGroupBox, QMessageBox, QCheckBox)
from PyQt5.QtCore import Qt, QTimer, pyqtSignal
from PyQt5.QtGui import QPixmap, QImage

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import Image, JointState
from cv_bridge import CvBridge

# --- ROS 2 & MOVEIT ---
from moveit_msgs.action import MoveGroup, ExecuteTrajectory
from moveit_msgs.srv import GetCartesianPath
from moveit_msgs.msg import Constraints, JointConstraint, PositionConstraint, OrientationConstraint, BoundingVolume, CollisionObject, MoveItErrorCodes, RobotState
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose, Point, Quaternion
from visualization_msgs.msg import Marker, MarkerArray 
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener


CAMERA_TOPIC_NAME = '/camera/image_raw' 
ARM_GROUP_NAME = "arm"
GRIPPER_GROUP_NAME = "gripper"
BASE_FRAME = "world"      
TIP_FRAME = "gripper-base-link"


BOX_X = 0.633
BOX_Y = -0.150
TEACH_HOVER_Z = 0.727
TEACH_GRASP_Z = 0.565

# Singularity & Güvenlik Sınırları
MAX_REACH_RADIUS = 0.85 
SAFE_MIN_RADIUS = 0.20
# ----------------------

class CobotGUI(QMainWindow):
    error_signal = pyqtSignal(str) 

    def __init__(self):
        super().__init__()
        self.setWindowTitle("Cobot Pro - FINAL DEMO (v11.4)")
        self.setMinimumSize(1300, 850)

        self.safety_limits_active = False 
        self.current_joint_state = None

        user_home = os.path.expanduser("~")
        self.save_path = os.path.join(user_home, "Pictures", "Cobot_Screenshots")
        if not os.path.exists(self.save_path):
            os.makedirs(self.save_path)

        if not rclpy.ok(): rclpy.init(args=None)
        self.node = Node('cobot_gui_node')
        
        self.collision_pub = self.node.create_publisher(CollisionObject, '/collision_object', 10)
        self.marker_pub = self.node.create_publisher(MarkerArray, '/visualization_marker_array', 10)
        self.joint_sub = self.node.create_subscription(JointState, '/joint_states', self.joint_callback, 10)
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self.node)
        self.move_action_client = ActionClient(self.node, MoveGroup, 'move_action')
        self.execute_traj_client = ActionClient(self.node, ExecuteTrajectory, 'execute_trajectory')
        self.cartesian_client = self.node.create_client(GetCartesianPath, 'compute_cartesian_path')
        
        qos_profile = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=10)
        self.bridge = CvBridge()
        self.image_sub = self.node.create_subscription(Image, CAMERA_TOPIC_NAME, self.image_callback, qos_profile) 
        
        # --- ROS SPINNING THREAD (Wait Set Hatası Çözümü) ---
        self.executor = MultiThreadedExecutor()
        self.executor.add_node(self.node)
        self.ros_thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.ros_thread.start()
        # ----------------------------------------------------

        self.timer = QTimer()
        self.timer.timeout.connect(self.update_gui)
        self.timer.start(100) 

        self.error_signal.connect(self.show_error_popup)

        self.themes = {"Turuncu (Ana)": "#FF8C00", "Yeşil": "#00FF00", "Mavi": "#00BFFF"}
        
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        self.main_layout = QHBoxLayout()
        central_widget.setLayout(self.main_layout)
        self.init_ui()
        
        self.setStyleSheet("""
            QWidget { background-color: #2D2D2D; color: white; font-family: Segoe UI; }
            QLineEdit { background-color: #404040; border: 1px solid #555; padding: 5px; border-radius: 3px; }
            QPushButton { background-color: #FF8C00; color: #111; font-weight: bold; padding: 8px; border-radius: 5px; }
            QPushButton:hover { background-color: #FFA500; }
            QGroupBox { border: 1px solid #555; margin-top: 20px; font-weight: bold; font-size: 14px; }
            QGroupBox::title { subcontrol-origin: margin; left: 10px; padding: 0 5px; color: #BBB; }
            QTabWidget::pane { border: 1px solid #444; }
            QCheckBox { font-weight: bold; font-size: 16px; color: #00FF00; spacing: 10px; }
            QCheckBox::indicator { width: 20px; height: 20px; }
        """)

    def update_gui(self):
        self.update_current_pose()

    def joint_callback(self, msg):
        self.current_joint_state = msg

    def update_current_pose(self):
        try:
            if self.tf_buffer.can_transform(BASE_FRAME, TIP_FRAME, rclpy.time.Time()):
                t = self.tf_buffer.lookup_transform(BASE_FRAME, TIP_FRAME, rclpy.time.Time())
                x, y, z = t.transform.translation.x, t.transform.translation.y, t.transform.translation.z
                
                is_safe = self.chk_safety.isChecked()
                status_text = "AÇIK ✅" if is_safe else "KAPALI ❌ (RİSKLİ MOD)"
                color = "#00FF00" if is_safe else "#FF0000"
                
                self.lbl_current_pose.setText(f"📍 UÇ NOKTA:\nX: {x:.3f} m\nY: {y:.3f} m\nZ: {z:.3f} m\n\n🛡️ KORUMA: {status_text}")
                self.lbl_current_pose.setStyleSheet(f"font-size: 14px; font-weight: bold; color: {color}; border: 2px solid {color}; padding: 10px; margin-top: 10px;")
        except TransformException: pass

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            rgb_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
            h, w, ch = rgb_image.shape
            qt_image = QImage(rgb_image.data, w, h, ch * w, QImage.Format_RGB888)
            self.video_label.setPixmap(QPixmap.fromImage(qt_image).scaled(640, 480, Qt.KeepAspectRatio))
        except: pass

    def show_error_popup(self, message):
        QMessageBox.critical(self, "Sistem Uyarısı", message)

    def init_ui(self):
        left_panel = QGroupBox("Görselleştirme & Ortam")
        left_layout = QVBoxLayout(); left_panel.setLayout(left_layout)
        
        self.video_label = QLabel("Kamera Sinyali Bekleniyor...")
        self.video_label.setAlignment(Qt.AlignCenter)
        self.video_label.setMinimumSize(640, 480)
        self.video_label.setStyleSheet("background-color: black; border: 2px solid #FF8C00;")
        left_layout.addWidget(self.video_label)

        btn_layout = QHBoxLayout()
        self.btn_viz = QPushButton("📐 Çalışma Alanını Çiz")
        self.btn_viz.clicked.connect(self.draw_workspace_limits)
        self.btn_add_table = QPushButton("🛑 Masayı Ekle")
        self.btn_add_table.clicked.connect(self.add_scene_objects)
        btn_layout.addWidget(self.btn_viz); btn_layout.addWidget(self.btn_add_table)
        left_layout.addLayout(btn_layout)

        self.lbl_current_pose = QLabel("Konum Bekleniyor...")
        left_layout.addWidget(self.lbl_current_pose)
        self.main_layout.addWidget(left_panel, 60)

        right_panel = QGroupBox("Robot Kontrol Merkezi")
        right_layout = QVBoxLayout(); right_panel.setLayout(right_layout)

        # --- GÜVENLİK CHECKBOX ---
        self.chk_safety = QCheckBox("🛡️ Singularity/Güvenlik Koruması")
        self.chk_safety.setChecked(True) 
        self.chk_safety.stateChanged.connect(self.toggle_safety)
        right_layout.addWidget(self.chk_safety)

        self.tabs = QTabWidget()
        self.tab_fk = QWidget(); self.tab_ik = QWidget()
        self.tabs.addTab(self.tab_fk, "İleri Kinematik"); self.tabs.addTab(self.tab_ik, "Ters Kinematik")
        
        self.setup_fk_tab(); self.setup_ik_tab()
        right_layout.addWidget(self.tabs)
        self.main_layout.addWidget(right_panel, 40)

    def toggle_safety(self):
        if self.chk_safety.isChecked():
            self.draw_workspace_limits()
        else:
            self.lbl_status.setText("⚠️ DİKKAT: Güvenlik Devre Dışı!")

    def setup_fk_tab(self):
        layout = QFormLayout()
        self.joint_inputs = []
        for i in range(6): 
            inp = QLineEdit("0.0"); self.joint_inputs.append(inp); layout.addRow(f"Joint {i+1} (rad):", inp)
        
        b_home = QPushButton("🏠 HOME (Sıfırla)"); b_home.clicked.connect(self.go_home)
        b_send = QPushButton("Açıları Gönder (FK)"); b_send.clicked.connect(self.send_fk_goal)
        layout.addRow(b_home); layout.addRow(b_send)
        
        hbox = QHBoxLayout()
        b_open = QPushButton("El AÇ"); b_open.clicked.connect(lambda: self.control_gripper(True))
        b_close = QPushButton("El KAPAT"); b_close.clicked.connect(lambda: self.control_gripper(False))
        hbox.addWidget(b_open); hbox.addWidget(b_close)
        layout.addRow(hbox)

        self.lbl_fk_status = QLabel("Durum: Bekleniyor..."); layout.addRow(self.lbl_fk_status)
        self.tab_fk.setLayout(layout)

    def setup_ik_tab(self):
        layout = QFormLayout()
        # İSTEK ÜZERİNE GÜNCELLENDİ: Hepsi 0.0 olarak başlıyor
        self.input_x = QLineEdit("0.0")
        self.input_y = QLineEdit("0.0")
        self.input_z = QLineEdit("0.0") 
        layout.addRow("Hedef X:", self.input_x); layout.addRow("Hedef Y:", self.input_y); layout.addRow("Hedef Z:", self.input_z)
        b_go = QPushButton("Konuma Git (IK)"); b_go.clicked.connect(self.send_ik_goal)
        layout.addRow(b_go)
        self.lbl_status = QLabel("Hazır"); self.lbl_status.setStyleSheet("color: orange; font-weight: bold;")
        layout.addRow(self.lbl_status)
        self.tab_ik.setLayout(layout)

    def go_home(self):
        for inp in self.joint_inputs: inp.setText("0.0")
        self.send_fk_goal()

    def draw_workspace_limits(self):
        marker_array = MarkerArray()
        
        outer_sphere = Marker()
        outer_sphere.header.frame_id = BASE_FRAME; outer_sphere.id = 0; outer_sphere.type = Marker.SPHERE; outer_sphere.action = Marker.ADD
        outer_sphere.scale.x = MAX_REACH_RADIUS * 2; outer_sphere.scale.y = MAX_REACH_RADIUS * 2; outer_sphere.scale.z = MAX_REACH_RADIUS * 2
        outer_sphere.pose.position.z = 0.4; outer_sphere.color.a = 0.1; outer_sphere.color.b = 1.0; outer_sphere.color.g = 0.5
        marker_array.markers.append(outer_sphere)

        inner_sphere = Marker()
        inner_sphere.header.frame_id = BASE_FRAME; inner_sphere.id = 1; inner_sphere.type = Marker.SPHERE; inner_sphere.action = Marker.ADD
        inner_sphere.scale.x = SAFE_MIN_RADIUS * 2; inner_sphere.scale.y = SAFE_MIN_RADIUS * 2; inner_sphere.scale.z = SAFE_MIN_RADIUS * 2
        inner_sphere.pose.position.z = 0.2; inner_sphere.color.a = 0.3; inner_sphere.color.r = 1.0
        marker_array.markers.append(inner_sphere)

        self.marker_pub.publish(marker_array)
        self.safety_limits_active = True
        self.chk_safety.setChecked(True)
        self.lbl_status.setText("Sınırlar Çizildi & Güvenlik AKTİF!")

    def control_gripper(self, open_gripper):
        goal = MoveGroup.Goal(); goal.request.group_name = GRIPPER_GROUP_NAME
        if open_gripper: pos_f1 = 0.0; pos_f2 = 0.0
        else: pos_f1 = -0.025; pos_f2 = 0.025
        cons = Constraints()
        jc1 = JointConstraint(); jc1.joint_name = "finger-1-joint"; jc1.position = pos_f1; jc1.tolerance_above=0.01; jc1.tolerance_below=0.01; jc1.weight=1.0; cons.joint_constraints.append(jc1)
        jc2 = JointConstraint(); jc2.joint_name = "finger-2-joint"; jc2.position = pos_f2; jc2.tolerance_above=0.01; jc2.tolerance_below=0.01; jc2.weight=1.0; cons.joint_constraints.append(jc2)
        goal.request.goal_constraints.append(cons)
        self.move_action_client.send_goal_async(goal)

    def send_fk_goal(self):
        try: angles = [float(i.text()) for i in self.joint_inputs]
        except: return
        threading.Thread(target=self._fk_thread, args=(angles,)).start()

    def _fk_thread(self, angles):
        goal = MoveGroup.Goal(); goal.request.group_name = ARM_GROUP_NAME
        cons = Constraints()
        for i, a in enumerate(angles):
            jc = JointConstraint(); jc.joint_name = f"part-{i+1}-joint"; jc.position = a; jc.tolerance_above=0.01; jc.tolerance_below=0.01; jc.weight=1.0; cons.joint_constraints.append(jc)
        goal.request.goal_constraints.append(cons)
        future = self.move_action_client.send_goal_async(goal)
        # Basit bekleme
        while rclpy.ok() and not future.done(): time.sleep(0.1)
        
        try:
            res = future.result()
            if res.accepted: 
                self.lbl_fk_status.setText("✅ Gönderildi")
            else: 
                self.lbl_fk_status.setText("❌ Reddedildi")
        except Exception as e:
            self.lbl_fk_status.setText(f"❌ Hata: {str(e)}")

    def send_ik_goal(self):
        try: x,y,z = float(self.input_x.text()), float(self.input_y.text()), float(self.input_z.text())
        except: return
        threading.Thread(target=self.move_arm_ik, args=(x,y,z,False,True)).start()

    def move_arm_ik(self, x, y, z, allow_collision=False, show_popup=False):
        
        # 1. GÜVENLİK KONTROLÜ (AÇIKSA)
        if self.chk_safety.isChecked():
            dist = math.sqrt(x**2 + y**2 + (z - 0.2)**2)
            if dist > MAX_REACH_RADIUS:
                msg = f"GÜVENLİK DURDURMASI!\nHedef Menzil Dışı ({dist:.2f}m > {MAX_REACH_RADIUS}m)"
                self.lbl_status.setText(f"❌ {msg}")
                if show_popup: self.error_signal.emit(msg)
                return False
            if dist < SAFE_MIN_RADIUS:
                msg = "GÜVENLİK DURDURMASI!\nHedef Robota Çok Yakın"
                self.lbl_status.setText(f"❌ {msg}")
                if show_popup: self.error_signal.emit(msg)
                return False

        # 2. GÜVENLİK KAPALIYSA -> CARTESIAN PATH (Riskli Mod)
        if not self.chk_safety.isChecked():
            # Cartesian Servisi Çağır
            if not self.cartesian_client.wait_for_service(timeout_sec=1.0):
                self.lbl_status.setText("HATA: Cartesian Servisi Yok!")
                return False

            req = GetCartesianPath.Request()
            req.header.frame_id = BASE_FRAME; req.group_name = ARM_GROUP_NAME; req.link_name = TIP_FRAME
            
            rs = RobotState()
            if self.current_joint_state: rs.joint_state = self.current_joint_state
            req.start_state = rs
            
            pose = Pose(); pose.position.x=x; pose.position.y=y; pose.position.z=z
            pose.orientation.x=-0.707; pose.orientation.w=0.707
            req.waypoints = [pose]
            req.max_step = 0.01; req.jump_threshold = 0.0; req.avoid_collisions = True 

            future = self.cartesian_client.call_async(req)
            while not future.done(): time.sleep(0.1)
            
            result = future.result()
            fraction = result.fraction
            
            if len(result.solution.joint_trajectory.points) > 0:
                # Yarım Yörüngeyi Yürüt
                goal = ExecuteTrajectory.Goal(); goal.trajectory = result.solution
                self.execute_traj_client.send_goal_async(goal)
                
                if fraction < 1.0:
                    time.sleep(2.0)
                    msg = f"SİSTEM UYARISI (Güvenliksiz Mod):\n\nRobot hedefe ulaşamıyor!\nYolun sadece %{fraction*100:.1f}'i gidilebildi.\nSEBEP: Singularity (Tekillik) veya Fiziksel Limit.\nRobot yarı yolda DURDU!"
                    self.lbl_status.setText(f"⚠️ Yarıda Kaldı (%{fraction*100:.0f})")
                    if show_popup: self.error_signal.emit(msg)
                    return False
                else:
                    self.lbl_status.setText("✅ Varıldı (Riskli Mod)")
                    return True
            else:
                self.lbl_status.setText("❌ HATA: Yol Hiç Bulunamadı!")
                if show_popup: self.error_signal.emit("KRİTİK HATA: MoveIt başlangıç noktasından kıpırdayamadı (Singularity içinde olabilir).")
                return False

        # 3. STANDART PLANLAMA (Güvenlik Açıksa) - PTP
        else:
            goal = MoveGroup.Goal()
            goal.request.group_name = ARM_GROUP_NAME
            goal.request.num_planning_attempts = 10
            goal.request.allowed_planning_time = 5.0
            
            pcm = PositionConstraint(); pcm.header.frame_id = BASE_FRAME; pcm.link_name = TIP_FRAME; pcm.weight = 1.0
            cbox = BoundingVolume(); prim = SolidPrimitive(); prim.type = SolidPrimitive.BOX; prim.dimensions = [0.05, 0.05, 0.05]
            cbox.primitives.append(prim)
            pose = Pose(); pose.position.x=x; pose.position.y=y; pose.position.z=z; pose.orientation.w=1.0
            cbox.primitive_poses.append(pose); pcm.constraint_region = cbox
            
            ocm = OrientationConstraint(); ocm.header.frame_id = BASE_FRAME; ocm.link_name = TIP_FRAME
            ocm.orientation.x = -0.707; ocm.orientation.w = 0.707
            ocm.absolute_x_axis_tolerance = 0.1; ocm.absolute_y_axis_tolerance = 0.1; ocm.absolute_z_axis_tolerance = 3.14; ocm.weight = 1.0
            
            goal.request.goal_constraints.append(Constraints(position_constraints=[pcm], orientation_constraints=[ocm]))
            
            future = self.move_action_client.send_goal_async(goal)
            while not future.done(): time.sleep(0.1) # Thread içindeyiz, sorun yok
            
            try:
                res = future.result()
                if not res.accepted: 
                    self.lbl_status.setText("Plan Reddedildi!"); 
                    if show_popup: self.error_signal.emit("MOVEIT HATASI:\nPlanlama başarısız oldu (Singularity veya Çarpışma).")
                    return False
                
                # Executing Result Bekle
                res_future = res.get_result_async()
                while not res_future.done(): time.sleep(0.1)
                
                final = res_future.result().result
                if final.error_code.val == 1:
                    self.lbl_status.setText("✅ Varıldı")
                    return True
                else:
                    # --- DETAYLI HATA SÖZLÜĞÜ (TÜM KODLAR) ---
                    code = final.error_code.val
                    err_map = {
                        1: "BAŞARILI",
                        99999: "BİLİNMEYEN",
                        -1: "PLANLAMA BAŞARISIZ (Yol Yok / Ulaşılamaz)",
                        -2: "ZAMAN AŞIMI (Planlama Süresi Yetmedi)",
                        -3: "BAŞLANGIÇ DURUMU HATALI",
                        -4: "BAŞLANGIÇTA ÇARPIŞMA",
                        -5: "HEDEFTE ÇARPIŞMA",
                        -10: "BAŞLANGIÇTA ÇARPIŞMA (Detaylı)",
                        -11: "HEDEFTE ÇARPIŞMA (Detaylı)",
                        -12: "HEDEF KISITLAMASI İHLALİ (Çarpışma)",
                        -21: "GEÇERSİZ HAREKET PLANI",
                        -31: "İK ÇÖZÜMÜ YOK (Fiziksel Olarak Ulaşılamaz)"
                    }
                    err_msg = err_map.get(code, f"MOVEIT HATASI (Kod: {code})")
                    self.lbl_status.setText(f"❌ {err_msg}")
                    if show_popup: self.error_signal.emit(f"MOVEIT SİSTEM HATASI:\n{err_msg}")
                    return False
            except Exception as e:
                print(f"HATA: {e}")
                return False

    def add_scene_objects(self):
        floor = CollisionObject(); floor.header.frame_id = BASE_FRAME; floor.id = "zemin"
        p_floor = SolidPrimitive(); p_floor.type = SolidPrimitive.BOX; p_floor.dimensions = [3.0, 3.0, 0.02]
        pose_f = Pose(); pose_f.position.z = -0.05; pose_f.orientation.w = 1.0
        floor.primitives.append(p_floor); floor.primitive_poses.append(pose_f); floor.operation = CollisionObject.ADD
        self.collision_pub.publish(floor)

        table = CollisionObject(); table.header.frame_id = BASE_FRAME; table.id = "masa"
        p_table = SolidPrimitive(); p_table.type = SolidPrimitive.BOX; p_table.dimensions = [0.4, 0.8, 0.5]
        pose_t = Pose(); pose_t.position.x = 0.6; pose_t.position.y = 0.0; pose_t.position.z = 0.25; pose_t.orientation.w = 1.0
        table.primitives.append(p_table); table.primitive_poses.append(pose_t); table.operation = CollisionObject.ADD
        self.collision_pub.publish(table)
        
        box = CollisionObject(); box.header.frame_id = BASE_FRAME; box.id = "hedef_kutu"
        p_box = SolidPrimitive(); p_box.type = SolidPrimitive.BOX; p_box.dimensions = [0.03, 0.03, 0.03]
        pose_b = Pose(); pose_b.position.x = BOX_X; pose_b.position.y = BOX_Y; pose_b.position.z = 0.515; pose_b.orientation.w = 1.0
        box.primitives.append(p_box); box.primitive_poses.append(pose_b); box.operation = CollisionObject.ADD
        self.collision_pub.publish(box)
        self.lbl_status.setText("Ortam ve Kutu Eklendi")

def main(args=None):
    app = QApplication(sys.argv)
    window = CobotGUI()
    window.show()
    try: sys.exit(app.exec_())
    finally: rclpy.shutdown()

if __name__ == '__main__': main()
