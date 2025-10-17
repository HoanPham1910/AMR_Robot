from PyQt5 import QtCore, QtGui, QtWidgets
import math


class MotorWidget(QtWidgets.QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.angle = 0  # góc ban đầu của kim
        self.setMinimumSize(150, 150)
        self.setMaximumSize(300, 300)

    def setAngle(self, angle):
        """Cập nhật góc kim và vẽ lại"""
        self.angle = angle % 360
        self.update()

    def paintEvent(self, event):
        painter = QtGui.QPainter(self)
        painter.setRenderHint(QtGui.QPainter.Antialiasing)

        w = self.width()
        h = self.height()
        center = QtCore.QPointF(w / 2, h / 2)

        # Giảm kích thước mặt động cơ xuống còn 2/3 khung
        radius = (min(w, h) / 2 - 10) * (2 / 3)

        # --- Vẽ mặt động cơ ---
        grad = QtGui.QRadialGradient(center, radius)
        grad.setColorAt(0, QtGui.QColor("#444"))
        grad.setColorAt(1, QtGui.QColor("#222"))
        painter.setBrush(QtGui.QBrush(grad))
        painter.setPen(QtGui.QPen(QtGui.QColor("#666"), 2))
        painter.drawEllipse(center, radius, radius)

        # --- Vẽ trục ---
        shaft_radius = radius * 0.1
        painter.setBrush(QtGui.QBrush(QtGui.QColor("#bbb")))
        painter.setPen(QtCore.Qt.NoPen)
        painter.drawEllipse(center, shaft_radius, shaft_radius)

        # --- Vẽ kim ---
        painter.save()
        painter.translate(center)
        painter.rotate(-self.angle)  # kim quay ngược chiều kim đồng hồ
        pen = QtGui.QPen(QtGui.QColor("#ff3333"), 3)
        pen.setCapStyle(QtCore.Qt.RoundCap)
        painter.setPen(pen)
        painter.drawLine(QtCore.QPointF(0, 0), QtCore.QPointF(0, -radius * 0.8))
        painter.restore()

        # --- Vẽ chữ góc ---
        painter.setPen(QtGui.QPen(QtGui.QColor("black")))
        painter.setFont(QtGui.QFont("Arial", 10, QtGui.QFont.Bold))
        painter.drawText(10, 20, f"Angle: {self.angle:.1f}°")

        painter.end()
