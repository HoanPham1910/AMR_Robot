from PyQt5 import QtWidgets, QtCore
from UI.test import Ui_Form
from Design_Frame.realtime_chart import RealtimeChart
from Library.ezi_servo_controller import EziServoController
import threading, time, math
import rclpy
from rclpy.executors import MultiThreadedExecutor

# --- ROS2 nodes (bạn đã có) ---
from publisher_cmd_vel import ROS2Publisher
from encoder_joint_state_publisher import EncoderJointStatePublisher
from odom_publisher import OdomPublisher
from tf_broadcaster import OdomTFBroadcaster


class MainWindow(QtWidgets.QWidget):
    vel_update_signal = QtCore.pyqtSignal(float, float)

    def __init__(self, ros_executor):
        super().__init__()
        self.ui = Ui_Form()
        self.ui.setupUi(self)

        self.ros_executor = ros_executor
        self.servo1 = None
        self.servo2 = None
        self.ros_encoder_node = None

        # --- Biểu đồ tốc độ ---
        self.chart_widget = RealtimeChart()
        layout = QtWidgets.QVBoxLayout()
        layout.addWidget(self.chart_widget)
        self.ui.groupBox_vel_graph.setLayout(layout)

        # --- Kết nối nút ---
        self.ui.btn_connect_ip.clicked.connect(self.connect_ip)
        self.ui.btn_set.clicked.connect(self.on_set_clicked)
        self.ui.btn_mode.clicked.connect(self.toggle_mode)
        self.ui.btn_stop.clicked.connect(self.stop_motor)
        self.ui.txt_R.textChanged.connect(self.on_radius_changed)

        # --- Khởi tạo signal ---
        self.vel_update_signal.connect(self.update_velocity_gui)

        # Thread control
        self.velocity_thread = None
        self.velocity_stop_event = None
        self.rpm_target = 0  # RPM mặc định
        self.current_direction = 1

    # -------------------------------------------------------
    def connect_ip(self):
        if self.ui.btn_connect_ip.text() == "Connect":
            ip1 = self.ui.txt_ip_1.text().strip()
            ip2 = self.ui.txt_ip_2.text().strip()
            port = 2001

            if not ip1 or not ip2:
                QtWidgets.QMessageBox.warning(self, "Cảnh báo", "Vui lòng nhập đủ 2 địa chỉ IP!")
                return

            try:
                # --- Kết nối servo ---
                self.servo1 = EziServoController(ip1, port)
                self.servo2 = EziServoController(ip2, port)
                QtWidgets.QMessageBox.information(self, "Thông báo", f"✅ Đã kết nối:\n{ip1}\n{ip2}")
                self.ui.btn_connect_ip.setText("Connected")

                # --- Tạo ROS2 node encoder sau khi kết nối ---
                if not self.ros_encoder_node:
                    self.ros_encoder_node = EncoderJointStatePublisher(self.servo1, self.servo2)
                    self.ros_executor.add_node(self.ros_encoder_node)

            except Exception as e:
                QtWidgets.QMessageBox.critical(self, "Lỗi kết nối", str(e))
                self.servo1 = None
                self.servo2 = None

        else:
            # Ngắt kết nối servo
            for s in [self.servo1, self.servo2]:
                if s:
                    s.stop()
                    s.close()
            self.servo1 = None
            self.servo2 = None
            QtWidgets.QMessageBox.information(self, "Thông báo", "Đã ngắt kết nối.")
            self.ui.btn_connect_ip.setText("Connect")

    # -------------------------------------------------------
    def on_set_clicked(self):
        mode = self.ui.btn_mode.text().lower()
        if mode == "velocity":
            self.set_velocity()
        elif mode == "position":
            self.set_position()

    def set_velocity(self):
        if not self.servo1 or not self.servo2:
            QtWidgets.QMessageBox.warning(self, "Cảnh báo", "Chưa kết nối servo!")
            return

        try:
            rpm = float(self.ui.txt_vel_rpm.text())
        except ValueError:
            QtWidgets.QMessageBox.warning(self, "Lỗi", "RPM không hợp lệ!")
            return

        self.rpm_target = rpm
        self.current_direction = 1 if rpm >= 0 else 0

        # Cập nhật trực tiếp ROS2 node
        if hasattr(self, 'ros_cmd_node'):
            self.ros_cmd_node.current_rpm = rpm

        # Khởi động thread 1 lần duy nhất
        if not self.velocity_thread or not self.velocity_thread.is_alive():
            self.velocity_stop_event = threading.Event()
            self.velocity_thread = threading.Thread(target=self.velocity_loop, daemon=True)
            self.velocity_thread.start()


    # -------------------------------------------------------
    def velocity_loop(self):
        stop_event = self.velocity_stop_event
        motors = [m for m in [self.servo1, self.servo2] if m]
        sp_previous = {motor: 0 for motor in motors}

        # Khởi động với rpm mặc định
        for motor in motors:
            motor.stop()
            motor.set_servo_enable(True)
            direction = 1 if self.rpm_target >= 0 else 0
            motor.move_velocity_rpm(abs(self.rpm_target), direction)
            sp_previous[motor] = self.rpm_target

        while not stop_event.is_set():
            rpm = self.rpm_target
            for motor in motors:
                if rpm == 0:
                    motor.stop()
                    sp_previous[motor] = 0
                    continue
                direction = 1 if rpm >= 0 else 0
                if rpm != sp_previous[motor]:
                    motor.velocity_override_rpm(rpm)
                    sp_previous[motor] = rpm

            rpm1 = motors[0].get_actual_velocity_rpm() if len(motors) > 0 else 0
            rpm2 = motors[1].get_actual_velocity_rpm() if len(motors) > 1 else 0
            self.vel_update_signal.emit(rpm1, rpm2)
            time.sleep(0.1)

    # -------------------------------------------------------
    def stop_motor(self):
        if self.velocity_stop_event:
            self.velocity_stop_event.set()
        for s in [self.servo1, self.servo2]:
            if s:
                s.stop()
        self.velocity_thread = None

    # -------------------------------------------------------
    def update_velocity_gui(self, rpm1, rpm2):
        avg_rpm = (rpm1 + rpm2) / 2
        self.ui.SpeedGauge.setValue(int(avg_rpm))
        self.chart_widget.update_chart(rpm1, rpm2)

    # -------------------------------------------------------
    def toggle_mode(self):
        if self.ui.btn_mode.text() == "velocity":
            self.ui.btn_mode.setText("position")
            self.ui.txt_deg.setReadOnly(False)
        else:
            self.ui.btn_mode.setText("velocity")
            self.ui.txt_deg.setReadOnly(True)

    # -------------------------------------------------------
    def on_radius_changed(self, text):
        try:
            value = float(text)
            self.ui.wheelWidget.setRadius(value)
        except ValueError:
            pass

    # -------------------------------------------------------
    def set_position(self):
        if not self.servo1 or not self.servo2:
            QtWidgets.QMessageBox.warning(self, "Cảnh báo", "Chưa kết nối servo!")
            return

        for m in (self.servo1, self.servo2):
            m.set_servo_enable(True)
        time.sleep(0.5)

        try:
            angle_deg = float(self.ui.txt_deg.text())
            rpm_speed = float(self.ui.txt_vel_rpm.text())
            radius_cm = float(self.ui.txt_R.text())
        except ValueError:
            QtWidgets.QMessageBox.warning(self, "Lỗi", "Giá trị nhập không hợp lệ!")
            return

        one_turn = EziServoController.deg_to_pulses(angle_deg)
        length_cm = EziServoController.pulses_to_cm(one_turn, radius_cm)

        cur1 = self.servo1.get_actual_position()
        cur2 = self.servo2.get_actual_position()
        if cur1 == -1 or cur2 == -1:
            QtWidgets.QMessageBox.warning(self, "Lỗi", "Không đọc được vị trí hiện tại!")
            return

        target1 = cur1 + one_turn
        target2 = cur2 + one_turn

        t1 = threading.Thread(target=self.servo1.move_incremental, args=(one_turn, rpm_speed))
        t2 = threading.Thread(target=self.servo2.move_incremental, args=(one_turn, rpm_speed))
        t1.start(); t2.start()
        t1.join(); t2.join()

    # -------------------------------------------------------
    def wait_until_reached(self, servo, target, tolerance=10, timeout=10):
        start_time = time.time()
        while True:
            QtWidgets.QApplication.processEvents()
            cur = servo.get_actual_position()
            if abs(cur - target) <= tolerance:
                break
            if time.time() - start_time > timeout:
                break
            time.sleep(0.1)

    # -------------------------------------------------------
    def closeEvent(self, event):
        self.stop_motor()
        event.accept()


def main():
    import sys
    rclpy.init()
    ros_executor = MultiThreadedExecutor()

    app = QtWidgets.QApplication(sys.argv)
    window = MainWindow(ros_executor)
    window.show()

    # --- Khởi tạo các node ROS2 ---
    window.ros_cmd_node = ROS2Publisher(radius=0.05)   # lưu vào window
    window.ros_cmd_node.current_rpm = window.rpm_target
    ros_odom_node = OdomPublisher()

    for node in [window.ros_cmd_node, ros_odom_node]:
        ros_executor.add_node(node)

    # --- ROS2 spin trong thread riêng ---
    spin_thread = threading.Thread(target=ros_executor.spin, daemon=True)
    spin_thread.start()

    sys.exit(app.exec_())



if __name__ == "__main__":
    main()
