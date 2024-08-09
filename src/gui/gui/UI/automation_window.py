import json
import sys
import time

from PyQt5 import Qt
from PyQt5.QtCore import QPoint, QTimer
from PyQt5.QtGui import QPen, QPainter, QPolygon, QColor
from PyQt5.QtWidgets import QDialog, QApplication

from UI import automation


class AutoWindow(QDialog):
    def __init__(self):
        super().__init__()
        self.ui = automation.Ui_Dialog()
        self.ui.setupUi(self)
        with open('./config.json') as cfg:
            cfg_dit = json.load(cfg)
            self.profile = cfg_dit['profile']

        # 创建多边形
        self.points = QPolygon()
        self.points.append(QPoint(self.profile[0][0] * 10 +50, 450))
        for coord in self.profile:
            coord_1 = [coord[0] * 10 +50, 450 - coord[1] * 10]
            coord_2 = [coord[2] * 10 +50, 450 - coord[1] * 10]
            self.points.append(QPoint(coord_1[0], coord_1[1]))
            self.points.append(QPoint(coord_2[0], coord_2[1]))
        self.points.append(QPoint(self.profile[-1][-1] * 10 +50, 450))

        # 初始化要绘制的点列表
        self.points_to_draw = []

        # 初始化多边形和点的颜色
        self.polygon_color = QColor(25, 0, 0)  # 红色
        self.point_color = QColor(255, 0, 0)  # 绿色

        # 设置定时器
        # self.timer = QTimer(self)
        # self.timer.timeout.connect(self.update_polygon)
        # self.timer.start(10)  # 设置定时器间隔为1秒

    def update_polygon(self, position_list):
        # 更新要绘制的点
        # self.points_to_draw.clear()  # 清空要绘制的点列表
        # for coord in position_list:
        #     self.points_to_draw.append(QPoint(int(coord[0]) * 10, 450 - int(coord[1]) * 10))
        self.points_to_draw.append(QPoint(int(position_list[0] * 10)+50, 450 - int(position_list[1] * 10)))
        self.update()  # 触发重绘事件

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)

        # 绘制多边形
        painter.setBrush(self.polygon_color)
        painter.drawPolygon(self.points)

        # 绘制点
        painter.setPen(self.point_color)
        painter.setBrush(self.point_color)  # 设置点的颜色
        # painter.setRenderHint(QPainter.Antialiasing, True)  # 设置反锯齿
        # painter.setRenderHint(QPainter.HighQualityAntialiasing, True)  # 设置抗锯齿
        for point in self.points_to_draw:
            painter.drawEllipse(point, 1, 1)  # 绘制椭圆作为点，椭圆大小为5x5像素


if __name__ == '__main__':
    app = QApplication(sys.argv)
    auto = AutoWindow()
    auto.show()
    list = [[5, 10], [5.5, 10], [6, 11], [7, 15]]
    # auto.update_polygon(list)
    sys.exit(app.exec_())
