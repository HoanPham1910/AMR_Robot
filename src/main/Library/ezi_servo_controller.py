import socket
from typing import List

#=====================================================================
# PHẦN 1: ĐỊNH NGHĨA CÁC HẰNG SỐ
#=====================================================================
FRAME_HEADER = 0xAA

# Lệnh điều khiển vị trí
CMD_MOVE_ABS_POS       = 0x34
CMD_MOVE_INC_POS       = 0x35
CMD_POS_ABS_OVERRIDE   = 0x38
CMD_POS_INC_OVERRIDE   = 0x39

# Lệnh điều khiển vận tốc
CMD_MOVE_VELOCITY      = 0x37
CMD_VELOCITY_OVERRIDE  = 0x3A
CMD_MOVE_VELOCITY_Ex   = 0x82
CMD_SERVO_ALARM_RESET = 0x2B

# Lệnh điều khiển chung và trạng thái
CMD_MOVE_STOP          = 0x31
CMD_EMERGENCY_STOP     = 0x32
CMD_GET_ACTUAL_POS     = 0x53
CMD_GET_ACTUAL_VEL     = 0x55
CMD_CLEAR_POSITION     = 0x56
CMD_SERVO_ENABLE       = 0x2A

#=====================================================================
# PHẦN 2: LỚP EziServoController
#=====================================================================
class EziServoController:
    def __init__(self, ip: str, port: int):
        self.ip_address = ip
        self.port = port
        self.sync_no = 0
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.settimeout(1.0)
        self.sock.connect((self.ip_address, self.port))
        self.PPR = 10000  # xung trên 1 vòng
    
    # Hàm helper chuyển RPM → PPS
    def _rpm_to_pps(self, rpm: float) -> int:
        return int(rpm * self.PPR / 60)
    
    def _int_to_bytes(self, value: int) -> bytes:
        return value.to_bytes(4, byteorder='little', signed=True)

    def _send_frame(self, frame_type: int, data: bytes = b'') -> bytes:
        self.sync_no = (self.sync_no + 1) % 256
        length = 1 + 1 + 1 + len(data)  # SyncNo + Reserved + FrameType + data
        reserved = 0
        packet = bytes([FRAME_HEADER, length, self.sync_no, reserved, frame_type]) + data
        self.sock.sendall(packet)
        # Nhận phản hồi (giả định tối đa 64 byte)
        try:
            response = self.sock.recv(64)
            return response
        except socket.timeout:
            return b''

    #=================================================================
    # CÁC HÀM ĐIỀU KHIỂN VỊ TRÍ
    #=================================================================
    def deg_to_pulses(deg: float, ppr: int = 10000) -> int:
        return int(deg / 360 * ppr)
    
    def pulses_to_cm(pulses: int, radius_cm: float, ppr: int = 10000) -> float:
        return 2 * 3.1415926 * radius_cm * pulses / ppr
    
    def move_to_absolute(self, position: int, rpm: float) -> bool:
        speed_pps = self._rpm_to_pps(rpm)
        data = self._int_to_bytes(position) + self._int_to_bytes(speed_pps)
        resp = self._send_frame(CMD_MOVE_ABS_POS, data)
        return len(resp) > 0

    def move_incremental(self, distance: int, rpm: float) -> bool:
        speed_pps = self._rpm_to_pps(rpm)
        data = self._int_to_bytes(distance) + self._int_to_bytes(speed_pps)
        resp = self._send_frame(CMD_MOVE_INC_POS, data)
        return len(resp) > 0

    def get_actual_position(self) -> int:
        resp = self._send_frame(CMD_GET_ACTUAL_POS)
        print("Raw response:", list(resp))  # debug
        if len(resp) >= 10:
            # 4 byte vị trí thực tế cuối cùng
            pos_bytes = resp[6:10]  # little-endian
            return int.from_bytes(pos_bytes, byteorder='little', signed=True)
        return -1

    #=================================================================
    # CÁC HÀM ĐIỀU KHIỂN VẬN TỐC
    #=================================================================
    def move_velocity_pulse(self, speed: int, direction: int) -> bool:
        data = self._int_to_bytes(speed) + bytes([1 if direction > 0 else 0])
        resp = self._send_frame(CMD_MOVE_VELOCITY, data)
        return len(resp) > 0
    
    def move_velocity_rpm(self, rpm: float, direction: int, pulses_per_rev: int = 10000) -> bool:
        # đổi rpm -> pulses/s
        pulse_per_sec = int((rpm * pulses_per_rev) / 60)
        data = self._int_to_bytes(pulse_per_sec) + bytes([1 if direction > 0 else 0])
        resp = self._send_frame(CMD_MOVE_VELOCITY, data)
        return len(resp) > 0
    
    def velocity_override_rpm(self, rpm: float, pulses_per_rev: int = 10000) -> bool:
        pulse_per_sec = int((abs(rpm) * pulses_per_rev) / 60)
        data = self._int_to_bytes(pulse_per_sec)
        resp = self._send_frame(CMD_VELOCITY_OVERRIDE, data)
        return len(resp) > 0
    
    def move_velocity_ex_rpm(
        self,
        rpm: float,
        direction: int = 1,
        job: int = 1,
        flag: int = 2,
        accel_decel_time: int = 2000,
        pulses_per_rev: int = 10000
    ) -> bool:
        """Move Velocity Ex — điều khiển tốc độ mượt theo RPM"""
        # --- Chuyển RPM → PPS ---
        pps = int((abs(rpm) * pulses_per_rev) / 60)

        # --- Tạo frame dữ liệu 37 byte ---
        data = (
            self._int_to_bytes(pps) +               # 4 byte tốc độ
            bytes([1 if direction > 0 else 0]) +    # 1 byte hướng
            self._int_to_bytes(flag) +              # 4 byte flag
            accel_decel_time.to_bytes(2, 'little') +# 2 byte accel/decel
            bytes(26)                               # 26 byte reserved
        )

        resp = self._send_frame(CMD_MOVE_VELOCITY_Ex, data)
        return len(resp) > 0



    #=================================================================
    # CÁC HÀM ĐỌC TỐC ĐỘ
    #=================================================================

    def get_actual_velocity(self) -> int:
        """
        Đọc tốc độ thực tế từ encoder (pps - pulses per second).
        """
        resp = self._send_frame(0x55, b"")
        if len(resp) >= 9:
            # tốc độ nằm ở byte [5:9]
            vel_pps = int.from_bytes(resp[5:9], byteorder="little", signed=True)
            return vel_pps
        return 0

    def get_actual_velocity_rpm(self, pulses_per_rev: int = 6397440) -> float:
        vel_pps = self.get_actual_velocity()
        return (vel_pps * 60.0) / pulses_per_rev



    #=================================================================
    # CÁC HÀM TIỆN ÍCH
    #=================================================================
    def stop(self) -> bool:
        resp = self._send_frame(CMD_MOVE_STOP)
        return len(resp) > 0

   
    def set_servo_enable(self, enable: bool) -> bool:
        data = bytes([1 if enable else 0])
        resp = self._send_frame(CMD_SERVO_ENABLE, data)
        return len(resp) > 0

    def close(self):
        self.sock.close()
    def reset_alarm(self):
        """Gửi lệnh reset alarm cho driver"""
        resp = self._send_frame(CMD_SERVO_ALARM_RESET, b"")
        if len(resp) > 0:
            print("✅ Alarm reset thành công.")
            return True
        else:
            print("⚠️ Alarm reset thất bại hoặc không phản hồi.")
            return False
    #=================================================================
    # HÀM CLEAR POSITION
    #=================================================================
    def clear_position(self) -> bool:
        """
        Reset vị trí encoder hiện tại về 0 trên driver.
        Trả về True nếu thành công, False nếu không phản hồi.
        """
        resp = self._send_frame(CMD_CLEAR_POSITION)
        if len(resp) > 0:
            print("✅ Clear position thành công.")
            return True
        else:
            print("⚠️ Clear position thất bại hoặc không phản hồi.")
            return False

