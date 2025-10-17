from PyQt5 import QtWidgets, QtCore
from UI.test import Ui_Form
from Design_Frame.speedometer import SpeedGauge
from Design_Frame.wheel_widget import WheelWidget
from Design_Frame.motor_widget import MotorWidget
from PyQt5.QtWidgets import QMessageBox
import threading
from threading import Thread  
import time, math
import socket
from Design_Frame.realtime_chart import RealtimeChart
from Library.ezi_servo_controller import EziServoController

class MainWindow(QtWidgets.QWidget):
    vel_update_signal = QtCore.pyqtSignal(float, float)
    def __init__(self):
        super().__init__()
        self.ui = Ui_Form()
        self.ui.setupUi(self)
        self.servo = None  # giữ object servo

        # --- Biểu đồ tốc độ ---
        self.chart_widget = RealtimeChart()
        layout = QtWidgets.QVBoxLayout()
        layout.addWidget(self.chart_widget)
        self.ui.groupBox_vel_graph.setLayout(layout)

        # --- Khởi tạo SpeedGauge ---
        self.ui.SpeedGauge.setRange(0, 3000)
        self.ui.SpeedGauge.setValue(0)

        # --- Wheel widget ---
        self.ui.wheelWidget.setRadius(5.0)
        self.ui.txt_R.textChanged.connect(self.on_radius_changed)

        # --- Biến shared ---
        self.current_rpm = 0
        self.current_direction = 1
        self.velocity_thread = None
        self.velocity_stop_event = None
        self.rpm_target = 100  # RPM mặc định
        # --- Kết nối nút ---
        self.ui.btn_connect_ip.clicked.connect(self.connect_ip)
        self.ui.btn_set.clicked.connect(self.on_set_clicked)
        self.ui.btn_mode.clicked.connect(self.toggle_mode)
        self.ui.btn_stop.clicked.connect(self.stop_motor)
        # --- Thêm nút chọn đồ thị ---
        self.ui.btn_m1.clicked.connect(lambda: self.chart_widget.show_series(1))
        self.ui.btn_m2.clicked.connect(lambda: self.chart_widget.show_series(2))
        self.ui.btn_m1.setCheckable(True)
        self.ui.btn_m2.setCheckable(True)
        self.ui.btn_m1.toggled.connect(self.toggle_chart_display)
        self.ui.btn_m2.toggled.connect(self.toggle_chart_display)

        # --- Khởi tạo signal ---
        self.vel_update_signal.connect(self.update_velocity_gui)
    def toggle_chart_display(self):
        if self.ui.btn_m1.isChecked() and not self.ui.btn_m2.isChecked():
            self.chart_widget.show_series(1)
        elif self.ui.btn_m2.isChecked() and not self.ui.btn_m1.isChecked():
            self.chart_widget.show_series(2)
        else:
            self.chart_widget.show_series(0)  # hiển thị cả 2
    def stop_motor(self):
        """Dừng động cơ và tạm dừng cập nhật biểu đồ"""
        if hasattr(self, "velocity_stop_event") and self.velocity_stop_event:
            self.velocity_stop_event.set()  # Dừng thread đọc tốc độ

        for servo in [getattr(self, 'servo1', None), getattr(self, 'servo2', None)]:
            if servo:
                try:
                    servo.stop()
                except Exception as e:
                    print(f"⚠️ Lỗi khi stop servo: {e}")
        self.velocity_thread = None  # Cho phép start lại lần sau
    def update_velocity_gui(self, rpm1, rpm2):
        """Cập nhật GUI từ thread an toàn"""
        avg_rpm = (rpm1 + rpm2) / 2
        self.ui.SpeedGauge.setValue(int(avg_rpm))
        self.chart_widget.update_chart(rpm1, rpm2)
    def connect_ip(self):
        if self.ui.btn_connect_ip.text() == "Connect":
            ip1 = self.ui.txt_ip_1.text().strip()
            ip2 = self.ui.txt_ip_2.text().strip()
            port = 2001

            # --- Kiểm tra dữ liệu nhập ---
            if not ip1 or not ip2:
                QMessageBox.warning(self, "Cảnh báo", "Vui lòng nhập đủ 2 địa chỉ IP trước khi kết nối!")
                return

            try:
                # --- Kết nối cả 2 servo ---
                self.servo1 = EziServoController(ip1, port)
                self.servo2 = EziServoController(ip2, port)
                self.servo = self.servo1 
                QMessageBox.information(
                    self,
                    "Thông báo",
                    f"✅ Đã kết nối thành công:\n- {ip1}:{port}\n- {ip2}:{port}"
                )
                self.ui.btn_connect_ip.setText("Connected")

            except (socket.timeout, ConnectionRefusedError, OSError) as e:
                QMessageBox.critical(
                    self,
                    "Lỗi",
                    f"Không kết nối được đến:\n- {ip1}:{port}\n- {ip2}:{port}\n\nChi tiết: {str(e)}"
                )
                self.servo1 = None
                self.servo2 = None

        else:
            # --- Ngắt kết nối ---
            for servo in [getattr(self, 'servo1', None), getattr(self, 'servo2', None)]:
                if servo:
                    servo.stop()
                    servo.close()

            self.servo1 = None
            self.servo2 = None

            QMessageBox.information(self, "Thông báo", "Đã ngắt kết nối cả hai servo.")
            self.ui.btn_connect_ip.setText("Connect")
    def on_set_clicked(self):
        # self.chart_widget.start_chart()
        mode = self.ui.btn_mode.text().lower()
        if mode == "velocity":
            self.set_velocity()
        elif mode == "position":
            self.set_position()
    def set_velocity(self):
        if not self.servo1 or not self.servo2:
            QMessageBox.warning(self, "Cảnh báo", "Chưa kết nối servo!")
            return
        
        try:
            rpm = float(self.ui.txt_vel_rpm.text())
        except ValueError:
            QMessageBox.warning(self, "Lỗi", "RPM không hợp lệ!")
            return

        self.rpm_target = rpm  # cập nhật RPM mà thread đọc
        self.current_direction = 1 if rpm >= 0 else 0

        # Khởi động thread 1 lần duy nhất
        if not hasattr(self, "velocity_thread") or not self.velocity_thread:
            self.velocity_stop_event = threading.Event()
            self.velocity_thread = Thread(target=self.velocity_loop, daemon=True)
            self.velocity_thread.start()
        # nếu thread đã chạy, chỉ cập nhật self.rpm_target, không tạo thread mới


    def velocity_loop(self, rpm_default=100):
        stop_event = self.velocity_stop_event
        motors = [m for m in [self.servo1, self.servo2] if m]

        # Khởi động với rpm mặc định
        sp_previous = {}
        for motor in motors:
            motor.stop()
            motor.set_servo_enable(True)
            direction = 1 if rpm_default >= 0 else 0
            motor.move_velocity_rpm(abs(rpm_default), direction=direction)
            print(f"▶️ Motor khởi động với RPM mặc định: {rpm_default} (dir={direction})")
            sp_previous[motor] = rpm_default  # lưu RPM trước đó riêng cho motor

        while not stop_event.is_set():
            rpm = self.rpm_target  # thread luôn đọc RPM mới nhất

            for motor in motors:
                if rpm == 0:
                    motor.stop()
                    sp_previous[motor] = 0
                    continue

                direction = 1 if rpm >= 0 else 0

                if rpm != sp_previous[motor]:
                    motor.velocity_override_rpm(rpm )
                    sp_previous[motor] = rpm
             # --- Cập nhật biểu đồ GUI ---
            rpm1 = motors[0].get_actual_velocity_rpm() if len(motors) > 0 else 0
            rpm2 = motors[1].get_actual_velocity_rpm() if len(motors) > 1 else 0
            self.vel_update_signal.emit(rpm1, rpm2)
            time.sleep(0.1)


    def toggle_mode(self):
        if self.ui.btn_mode.text() == "velocity":
            self.ui.btn_mode.setText("position")
            self.ui.txt_deg.setReadOnly(False)
            self.ui.txt_deg.setStyleSheet("")
            self.ui.txt_R.setReadOnly(False)
            self.ui.txt_R.setStyleSheet("")
        else:
            self.ui.btn_mode.setText("velocity")
            self.ui.txt_deg.setReadOnly(True)
            self.ui.txt_deg.setStyleSheet("background-color: lightgray;")
            self.ui.txt_R.setReadOnly(True)
            self.ui.txt_R.setStyleSheet("background-color: lightgray;")
    def on_radius_changed(self, text):
        try:
            value = float(text)
            self.ui.wheelWidget.setRadius(value)
        except ValueError:
            pass
    def keyPressEvent(self, event):
        if event.key() == QtCore.Qt.Key_Escape:
            QtWidgets.qApp.quit()
    def set_position(self):
        if not hasattr(self, "servo1") or not hasattr(self, "servo2") or not self.servo1 or not self.servo2:
            QMessageBox.warning(self, "Cảnh báo", "Chưa kết nối servo!")
            return

        for m in (self.servo1, self.servo2):
            m.set_servo_enable(True)
        time.sleep(1)

        try:
            # === Lấy thông số từ GUI ===
            angle_deg = float(self.ui.txt_deg.text())
            rpm_speed = float(self.ui.txt_vel_rpm.text())
            radius_cm = float(self.ui.txt_R.text())
        except ValueError:
            QMessageBox.warning(self, "Lỗi", "Giá trị nhập không hợp lệ!")
            return

        # === Tính xung & quãng đường ===
        one_turn = EziServoController.deg_to_pulses(angle_deg)
        length_cm = EziServoController.pulses_to_cm(one_turn, radius_cm)

        # === Lấy vị trí hiện tại ===
        cur1 = self.servo1.get_actual_position()
        cur2 = self.servo2.get_actual_position()

        if cur1 == -1 or cur2 == -1:
            QMessageBox.warning(self, "Lỗi", "Không đọc được vị trí hiện tại!")
            for m in (self.servo1, self.servo2):
                m.set_servo_enable(False)
            return

        target1 = cur1 + one_turn
        target2 = cur2 + one_turn

        print(f"Quay {angle_deg}° → {one_turn} xung → {length_cm:.2f} cm")
        print(f"Motor1: {cur1}→{target1} | Motor2: {cur2}→{target2} | {rpm_speed} RPM")

        # === Quay song song ===
        t1 = threading.Thread(target=self.servo1.move_incremental, args=(one_turn, rpm_speed))
        t2 = threading.Thread(target=self.servo2.move_incremental, args=(one_turn, rpm_speed))
        t1.start(); t2.start()
        t1.join(); t2.join()

        # === Chờ đạt vị trí ===
        self.wait_until_reached(self.servo1, target1)
        self.wait_until_reached(self.servo2, target2)

        print(f"✅ Đã quay xong {angle_deg}° (≈{length_cm:.2f} cm) cho cả 2 motor!")

        for m in (self.servo1, self.servo2):
            m.set_servo_enable(False)
        print("🔌 Đã tắt servo cả 2 motor.")
    def wait_until_reached(self, servo, target, tolerance=10, timeout=10):
        start_time = time.time()
        while True:
            QtWidgets.QApplication.processEvents()  # cho phép GUI cập nhật
            cur = servo.get_actual_position()
            if abs(cur - target) <= tolerance:
                break
            if time.time() - start_time > timeout:
                print(f"⏱️ Timeout: servo chưa đạt {target}")
                break
            time.sleep(0.1)
    def closeEvent(self, event):
        self.stop_motor()
        event.accept()


if __name__ == "__main__":
    import sys
    app = QtWidgets.QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec_())
