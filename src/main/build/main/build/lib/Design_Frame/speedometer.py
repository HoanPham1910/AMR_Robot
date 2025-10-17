from PyQt5.QtWidgets import QWidget
from PyQt5.QtGui import QPainter, QPen, QBrush, QFont
from PyQt5.QtCore import Qt, QRectF, QPointF
import math

class SpeedGauge(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self._value = 0
        self._min = 0
        self._max = 3000
        self._unit = "RPM"

    def setRange(self, min_val, max_val):
        self._min = min_val
        self._max = max_val
        self.update()

    def setValue(self, value):
        self._value = max(self._min, min(value, self._max))
        self.update()

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)

        w = self.width()
        h = self.height()
        side = min(w, h)
        painter.translate(w / 2, h / 2)
        painter.scale(side / 200.0, side / 200.0)

        # Vẽ nền
        painter.setBrush(QBrush(Qt.black))
        painter.setPen(Qt.NoPen)
        painter.drawEllipse(QPointF(0, 0), 100, 100)

        # Vẽ vạch chia
        painter.setPen(QPen(Qt.white, 2))
        for i in range(0, 181, 10):
            angle = 180 - i
            rad = math.radians(angle)
            x1 = 80 * math.cos(rad)
            y1 = -80 * math.sin(rad)
            x2 = 90 * math.cos(rad)
            y2 = -90 * math.sin(rad)
            painter.drawLine(int(x1), int(y1), int(x2), int(y2))

        # Vẽ kim
        painter.setPen(QPen(Qt.red, 4))
        painter.setBrush(Qt.red)
        ratio = (self._value - self._min) / (self._max - self._min)
        angle = 180 * ratio
        rad = math.radians(180 - angle)
        x = 70 * math.cos(rad)
        y = -70 * math.sin(rad)
        painter.drawLine(0, 0, int(x), int(y))

        # Vẽ tâm
        painter.setBrush(Qt.white)
        painter.setPen(Qt.NoPen)
        painter.drawEllipse(QPointF(0, 0), 5, 5)

        # Hiển thị giá trị
        painter.setPen(Qt.white)
        painter.setFont(QFont("Arial", 12))
        painter.drawText(-25, 50, f"{int(self._value)} {self._unit}")

        painter.setFont(QFont("Arial", 10))
        painter.drawText(-20, 65, f"{self._min}")
        painter.drawText(10, 65, f"{self._max}")

    def sizeHint(self):
        return self.minimumSizeHint()

    def minimumSizeHint(self):
        return self.size()
