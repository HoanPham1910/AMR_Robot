from PyQt5 import QtWidgets, QtCore, QtChart, QtGui
import sys, time, random

class ZoomableChartView(QtChart.QChartView):
    """ChartView hỗ trợ zoom + crosshair + click-to-show-coords + dynamic target marker"""
    def __init__(self, chart, owner=None, parent=None):
        super().__init__(chart, parent)
        self.owner = owner
        self.setRenderHint(QtGui.QPainter.Antialiasing)
        self.setMouseTracking(True)
        self.setRubberBand(QtChart.QChartView.RectangleRubberBand)

        pen = QtGui.QPen(QtGui.QColor(0, 0, 0, 120))
        pen.setStyle(QtCore.Qt.DashLine)
        pen.setWidth(1)

        self._vline = QtWidgets.QGraphicsLineItem()
        self._vline.setPen(pen)
        self._hline = QtWidgets.QGraphicsLineItem()
        self._hline.setPen(pen)

        self._marker = None
        self._marker_pen = QtGui.QPen(QtGui.QColor(200, 20, 20, 220))
        self._marker_brush = QtGui.QBrush(QtGui.QColor(255, 200, 200, 180))

        self.target_point = None  # lưu point đã click để marker dynamic

        sc = self.scene()
        sc.addItem(self._vline)
        sc.addItem(self._hline)

    def wheelEvent(self, event: QtGui.QWheelEvent):
        zoom_factor = 1.2
        if event.angleDelta().y() > 0:
            self.chart().zoom(zoom_factor)
        else:
            self.chart().zoom(1 / zoom_factor)
        event.accept()

    def mouseMoveEvent(self, event):
        scene_pos = self.mapToScene(event.pos())
        try:
            series = self.owner.series1 if self.owner.series1.isVisible() else self.owner.series2
            val = self.chart().mapToValue(scene_pos, series)
            x_val, y_val = val.x(), val.y()

            axis_x = self.chart().axisX()
            axis_y = self.chart().axisY()
            xmin, xmax = axis_x.min(), axis_x.max()
            ymin, ymax = axis_y.min(), axis_y.max()

            top_pt = self.chart().mapToPosition(QtCore.QPointF(x_val, ymax), series)
            bottom_pt = self.chart().mapToPosition(QtCore.QPointF(x_val, ymin), series)
            left_pt = self.chart().mapToPosition(QtCore.QPointF(xmin, y_val), series)
            right_pt = self.chart().mapToPosition(QtCore.QPointF(xmax, y_val), series)

            self._vline.setLine(top_pt.x(), top_pt.y(), bottom_pt.x(), bottom_pt.y())
            self._hline.setLine(left_pt.x(), left_pt.y(), right_pt.x(), right_pt.y())

            QtWidgets.QToolTip.showText(event.globalPos(), f"x={x_val:.2f}s\ny={y_val:.1f} RPM", self)
        except Exception:
            QtWidgets.QToolTip.hideText()

        super().mouseMoveEvent(event)

    def mousePressEvent(self, event):
        if event.button() == QtCore.Qt.LeftButton:
            scene_pos = self.mapToScene(event.pos())
            try:
                series = self.owner.series1 if self.owner.series1.isVisible() else self.owner.series2
                val = self.chart().mapToValue(scene_pos, series)
                x_click, y_click = val.x(), val.y()
            except Exception:
                return super().mousePressEvent(event)

            data = self.owner.data1 if series == self.owner.series1 else self.owner.data2
            if not data:
                return super().mousePressEvent(event)

            nearest = min(data, key=lambda p: (p[0] - x_click) ** 2 + (p[1] - y_click) ** 2)
            xi, yi = nearest
            self.target_point = nearest  # lưu lại target để dynamic marker

            pos_scene = self.chart().mapToPosition(QtCore.QPointF(xi, yi), series)

            if self._marker is not None:
                self.scene().removeItem(self._marker)
                self._marker = None

            r = 6
            ellipse = QtWidgets.QGraphicsEllipseItem(pos_scene.x() - r/2, pos_scene.y() - r/2, r, r)
            ellipse.setPen(self._marker_pen)
            ellipse.setBrush(self._marker_brush)
            ellipse.setZValue(10)
            self.scene().addItem(ellipse)
            self._marker = ellipse

            QtWidgets.QToolTip.showText(event.globalPos(),
                                        f"Nearest point:\n x={xi:.2f}s\n y={yi:.1f} RPM",
                                        self)

        return super().mousePressEvent(event)

    def update_marker_position(self):
        """Cập nhật marker khi dữ liệu thay đổi"""
        if not self.target_point or not self._marker:
            return
        xi, yi = self.target_point
        series = self.owner.series1 if self.owner.series1.isVisible() else self.owner.series2
        # tìm lại điểm gần nhất trong dữ liệu mới
        data = self.owner.data1 if series == self.owner.series1 else self.owner.data2
        if not data:
            return
        nearest = min(data, key=lambda p: (p[0] - xi) ** 2 + (p[1] - yi) ** 2)
        self.target_point = nearest
        xi, yi = nearest
        pos_scene = self.chart().mapToPosition(QtCore.QPointF(xi, yi), series)
        self._marker.setRect(pos_scene.x() - 3, pos_scene.y() - 3, 6, 6)


class RealtimeChart(QtWidgets.QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)

        self.series1 = QtChart.QLineSeries()
        self.series1.setName("RPM_1")
        pen1 = QtGui.QPen(QtGui.QColor("blue")); pen1.setWidth(2)
        self.series1.setPen(pen1)

        self.series2 = QtChart.QLineSeries()
        self.series2.setName("RPM_2")
        pen2 = QtGui.QPen(QtGui.QColor("orange")); pen2.setWidth(2)
        self.series2.setPen(pen2)

        self.chart = QtChart.QChart()
        self.chart.addSeries(self.series1)
        self.chart.addSeries(self.series2)

        self.axis_x = QtChart.QValueAxis()
        self.axis_y = QtChart.QValueAxis()
        self.axis_x.setTitleText("Time (s)")
        self.axis_y.setTitleText("Speed (RPM)")
        self.axis_x.setRange(0, 10)
        self.axis_y.setRange(0, 3500)
        self.axis_x.setTickCount(11)
        self.axis_y.setTickCount(15)

        self.chart.setAxisX(self.axis_x, self.series1)
        self.chart.setAxisY(self.axis_y, self.series1)
        self.chart.setAxisX(self.axis_x, self.series2)
        self.chart.setAxisY(self.axis_y, self.series2)

        self.chart.legend().setVisible(False)
        self.chart.setBackgroundBrush(QtGui.QBrush(QtGui.QColor("white")))

        self.chart_view = ZoomableChartView(self.chart, owner=self)
        self.chart_view.setStyleSheet("QChartView {background-color: white; border-radius: 8px;}")

        label_layout = QtWidgets.QHBoxLayout()
        label1 = QtWidgets.QLabel("● RPM Motor 1")
        label1.setStyleSheet("color: blue; font: bold 14px Arial;")
        label2 = QtWidgets.QLabel("● RPM Motor 2")
        label2.setStyleSheet("color: orange; font: bold 14px Arial;")
        label_layout.addStretch()
        label_layout.addWidget(label1)
        label_layout.addSpacing(20)
        label_layout.addWidget(label2)
        label_layout.addStretch()

        main_layout = QtWidgets.QVBoxLayout(self)
        main_layout.addWidget(self.chart_view)
        main_layout.addLayout(label_layout)
        main_layout.setContentsMargins(10, 10, 10, 2)

        self.start_time = time.time()
        self.max_points = 500
        self.data1 = []
        self.data2 = []

    def show_series(self, motor_id: int):
        if motor_id == 1:
            self.series1.setVisible(True)
            self.series2.setVisible(False)
        elif motor_id == 2:
            self.series1.setVisible(False)
            self.series2.setVisible(True)
        elif motor_id == 0:
            self.series1.setVisible(True)
            self.series2.setVisible(True)

    def visible_x_range(self):
        try:
            return (self.axis_x.min(), self.axis_x.max())
        except Exception:
            if self.data1:
                xs = [p[0] for p in self.data1]
                return (min(xs), max(xs))
            return (0, 10)

    def update_chart(self, rpm1, rpm2):
        elapsed_s = (time.time() - self.start_time)

        self.series1.append(elapsed_s, rpm1)
        self.series2.append(elapsed_s, rpm2)
        self.data1.append((elapsed_s, rpm1))
        self.data2.append((elapsed_s, rpm2))

        if len(self.data1) > self.max_points:
            remove_count = len(self.data1) - self.max_points
            self.series1.removePoints(0, remove_count)
            self.series2.removePoints(0, remove_count)
            self.data1 = self.data1[remove_count:]
            self.data2 = self.data2[remove_count:]

        # Scroll trục X theo thời gian
        min_x = self.series1.at(0).x()
        max_x = self.series1.at(self.series1.count() - 1).x()
        self.axis_x.setRange(min_x, max_x)

        # Update marker nếu có
        self.chart_view.update_marker_position()

    def start_chart(self):
        self.start_time = time.time()
        self.series1.clear()
        self.series2.clear()
        self.data1.clear()
        self.data2.clear()
