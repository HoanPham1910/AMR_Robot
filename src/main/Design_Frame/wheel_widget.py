from PyQt5.QtWidgets import QWidget
from PyQt5.QtGui import QPainter, QPen, QBrush, QColor, QFont
from PyQt5.QtCore import Qt, QRectF, QPointF
import math

class WheelWidget(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self._radius = 0
        self.setMinimumSize(150, 150)

    def setRadius(self, value):
        """Cập nhật bán kính (cm)"""
        self._radius = float(value)
        self.update()

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)

        w, h = self.width(), self.height()
        center_x, center_y = w / 2, h / 2 - 10
        radius_outer = min(w, h) / 2 - 30
        radius_inner = radius_outer * 0.6

        # --- Vẽ bánh xe ---
        # Lớp ngoài vàng
        painter.setBrush(QBrush(QColor(255, 204, 0)))  # vàng
        painter.setPen(QPen(Qt.black, 2))
        painter.drawEllipse(int(center_x - radius_outer), int(center_y - radius_outer),
                            int(radius_outer * 2), int(radius_outer * 2))

        # Lớp trong đen
        painter.setBrush(QBrush(Qt.black))
        painter.drawEllipse(int(center_x - radius_inner), int(center_y - radius_inner),
                            int(radius_inner * 2), int(radius_inner * 2))

        # --- Vẽ dim đỏ (giống AutoCAD) ---
        painter.setPen(QPen(QColor(255, 0, 0), 2))  # đỏ

        # Vẽ từ tâm ra mép (góc 45 độ)
        angle_deg = 45
        angle_rad = math.radians(angle_deg)

        end_x = center_x + radius_outer * math.cos(angle_rad)
        end_y = center_y - radius_outer * math.sin(angle_rad)
        

        painter.drawLine(int(center_x), int(center_y), int(end_x), int(end_y))

        # Vẽ mũi tên nhỏ ở đầu dim
        arrow_size = 8
        painter.setBrush(QBrush(QColor(255, 0, 0)))
        arrow_angle1 = angle_rad + math.radians(150)
        arrow_angle2 = angle_rad - math.radians(150)
        point1 = QPointF(end_x + arrow_size * math.cos(arrow_angle1),
                         end_y - arrow_size * math.sin(arrow_angle1))
        point2 = QPointF(end_x + arrow_size * math.cos(arrow_angle2),
                         end_y - arrow_size * math.sin(arrow_angle2))
        painter.drawPolygon(QPointF(end_x, end_y), point1, point2)

        # Vẽ chữ "R" đỏ bên cạnh dim
        painter.setPen(QColor(255, 0, 0))
        painter.setFont(QFont("Arial", 12, QFont.Bold))
        painter.drawText(int(end_x + 10), int(end_y - 10), "R")


        # --- Vẽ đế (dim hiển thị giá trị) ---
        base_rect = QRectF(center_x - 40, center_y + radius_outer - 5, 80, 30)
        painter.setBrush(QBrush(QColor(100, 100, 100)))  # xám
        painter.setPen(QPen(Qt.black, 1))
        painter.drawRoundedRect(base_rect, 6, 6)

        # --- Vẽ chữ "R = ..." ---
        painter.setPen(Qt.white)
        painter.setFont(QFont("Arial", 12))
        painter.drawText(base_rect, Qt.AlignCenter, f"R = {self._radius:.1f} cm")
