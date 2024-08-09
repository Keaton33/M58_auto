import threading
from UI import main_window, setting_window, alarm_window, comm_window, automation_window
from rclpy.node import Node
import rclpy
from interface.msg import Marker, PLC, Trolley
import rclpy.time
from sensor_msgs.msg import Image
import cv_bridge
from PyQt5.QtWidgets import QApplication, QMainWindow
from PyQt5.QtGui import QImage, QPixmap
from PyQt5.QtCore import QTimer
import sys
import cv2
import pyqtgraph as pg
import yaml


class Gui(Node):
    def __init__(self):
        super().__init__('gui_node')
        self.sub_marker = self.create_subscription(Marker, 'marker', self.cb_marker, 10)
        self.sub_camera = self.create_subscription(Image, 'ipcamera_marker', self.cb_camera, 10)
        self.sub_plc = self.create_subscription(PLC, 'plc', self.cb_plc, 10)
        self.sub_trolley = self.create_subscription(Trolley, 'trolley', self.cb_trolley, 10)
        self.marker = [0,0,0,0]
        self.t_y = 0
        self.dis_diff = 0
        self.t_y_ref = 0
        self.img = None

    def cb_marker(self, msg:Marker):
        self.marker = [msg.g_x, msg.t_y, msg.g_l, msg.t_l]
        self.t_y = msg.t_y
        self.dis_diff = msg.dis_diff
        self.t_y_ref = msg.t_y_ref
    
    def cb_camera(self, msg:Image):
        cvb = cv_bridge.CvBridge()
        self.img = cvb.imgmsg_to_cv2(msg)

    def cb_plc(self, msg:PLC):
        self.t_spd_cmd = msg.t_spd_cmd
        self.t_spd_act = msg.t_spd_act
        self.t_spd_set = msg.t_spd_set
        self.t_pos_act = msg.t_pos_act
        self.h_spd_cmd = msg.h_spd_cmd
        self.h_spd_act = msg.h_spd_act
        self.h_pos_act = msg.h_pos_act
        self.sc_on_off = msg.sc_on_off
        self.auto_on_off = msg.auto_on_off

    def cb_trolley(self, msg:Trolley):
        pass

class AutoAPP(QMainWindow):
    def __init__(self, mynode:Gui):
        QMainWindow.__init__(self)
        self.main_window = main_window.MainWindow()
        self.setting_window = setting_window.SettingWindow()
        self.alarm_window = alarm_window.AlarmWindow()
        self.comm_window = comm_window.CommWindow()
        self.main_window.ui.setupUi(self)
        # self.setting_window.ui.setupUi(self)
        # self.alarm_window.ui.setupUi(self)
        # self.comm_window.ui.setupUi(self)
        self.data1_1, self.data1_2, self.data1_3 = [], [], []
        self.data2_1, self.data2_2, self.data2_3 = [], [], []
        self.data3_1, self.data3_2 = [], []
        self.data_limit = 1000
        self.points = {}

        self.timer = QTimer()
        self.timer.timeout.connect(lambda: self.show_pix(image=mynode.img))
        self.timer.start(40)
        self.timer1 = QTimer()
        self.timer1.timeout.connect(lambda: self.show_curve(center_act=mynode.t_y, center_set=mynode.t_y_ref, dis_diff=mynode.dis_diff,\
                                                            t_spd_act=mynode.t_spd_act, t_spd_set=mynode.t_spd_set,t_spd_cmd=mynode.t_spd_cmd ))
        self.timer1.timeout.connect(lambda: self.show_data(center_act=mynode.t_y))
        self.timer1.start(100)

        self.setting_timer = QTimer()
        self.setting_timer.timeout.connect(lambda: self.set_points(mynode.h_pos_act, mynode.marker))
    
        # Create separate instances of PyGraphy trend chart for each QGroupBox
        self.trend_chart_1 = pg.PlotWidget()
        self.trend_curve1_1 = self.trend_chart_1.plot(pen='r')  # Red curve
        self.trend_curve1_2 = self.trend_chart_1.plot(pen='g')  # Green curve
        self.trend_curve1_3 = self.trend_chart_1.plot(pen='b')  # Green curve

        self.trend_chart_2 = pg.PlotWidget()
        self.trend_curve2_1 = self.trend_chart_2.plot(pen='r')  # Green curve
        self.trend_curve2_2 = self.trend_chart_2.plot(pen='g')  # Green curve
        self.trend_curve2_3 = self.trend_chart_2.plot(pen='b')  # Green curve

        self.trend_chart_3 = pg.PlotWidget()
        self.trend_curve3_1 = self.trend_chart_3.plot(pen='r')  # Blue curve
        self.trend_curve3_2 = self.trend_chart_3.plot(pen='g')  # Blue curve

        # Set the layout for the widget_trace QGroupBox
        self.main_window.ui.widget.layout().addWidget(self.trend_chart_1)
        self.main_window.ui.widget_2.layout().addWidget(self.trend_chart_2)
        self.main_window.ui.widget_3.layout().addWidget(self.trend_chart_3)

        self.main_window.ui.actionSettings.triggered.connect(self.setting_window.show)
        self.main_window.ui.actionComm.triggered.connect(self.comm_window.show)
        self.main_window.ui.actionAlarm.triggered.connect(self.alarm_window.show)
        self.setting_window.ui.pushButton_top.clicked.connect(lambda: self.start_point(mynode.h_pos_act, mynode.marker))
        self.setting_window.ui.pushButton_bottom.clicked.connect(lambda: self.stop_point(mynode.h_pos_act, mynode.marker))

    def show_pix(self, image):
        if image is not None:
            img = cv2.resize(image, (self.main_window.ui.label_img.size().width(),self.main_window.ui.label_img.size().height()))
            height, width, channel = img.shape
            bytes_per_line = 3 * width
            q_image = QImage(img.data, width, height, bytes_per_line, QImage.Format_RGB888).rgbSwapped()

            # Display the image in the QLabel
            pixmap = QPixmap.fromImage(q_image)
            self.main_window.ui.label_img.setPixmap(pixmap)

    def show_data(self, center_set=0, center_act=0, spd_cmd=0, spd_set=0, spd_act=0, skew_act=0.0, skew_set=0.0):
        self.main_window.ui.label_hbCenterSet.setText(str(center_set))
        self.main_window.ui.label_hbCenterAct.setText(str(center_act))
        self.main_window.ui.label_trolleySpdCmd.setText(str(skew_act))
        self.main_window.ui.label_trolleySpdSet.setText(str(spd_set))
        self.main_window.ui.label_trolleySpdAct.setText(str(spd_act))
        # self.main_window.ui.label_hbSkewAct.setText(str(skew_act))
        # self.main_window.ui.label_hbSkewSet.setText(str(skew_set))

    def show_curve(self, center_act=0, center_set=0, dis_diff=0, t_spd_set=0, t_spd_act=0, t_spd_cmd=0, h_spd_act=0, h_height=0):
        self.data1_1.append(center_act)
        self.data1_2.append(center_set)
        self.data1_3.append(dis_diff)
        self.data1_1 = self.data1_1[-self.data_limit:]
        self.data1_2 = self.data1_2[-self.data_limit:]
        self.data1_3 = self.data1_3[-self.data_limit:]
        self.trend_curve1_1.setData(self.data1_1)
        self.trend_curve1_2.setData(self.data1_2)
        self.trend_curve1_3.setData(self.data1_3)
        
        self.data2_1.append(t_spd_set/100)
        self.data2_2.append(t_spd_act)
        self.data2_3.append(t_spd_cmd)
        self.data2_1 = self.data2_1[-self.data_limit:]
        self.data2_2 = self.data2_2[-self.data_limit:]
        self.data2_3 = self.data2_3[-self.data_limit:]
        self.trend_curve2_1.setData(self.data2_1)
        self.trend_curve2_2.setData(self.data2_2)
        self.trend_curve2_3.setData(self.data2_3)

        self.data3_1.append(h_spd_act)
        self.data3_2.append(h_height)
        self.data3_1 = self.data3_1[-self.data_limit:]
        self.data3_2 = self.data3_2[-self.data_limit:]
        self.trend_curve3_1.setData(self.data3_1)
        self.trend_curve3_2.setData(self.data3_2)

    def closeEvent(self, event):
        rclpy.shutdown()

    def start_point(self, h_pos_act=0, marker=[0,0,0,0]):
        self.setting_window.ui.label_hoist_height_top.setText(str(h_pos_act))
        self.setting_window.ui.label_xyxy_top.setText(str(marker))
        self.setting_timer.start(100)

    def set_points(self, h_pos_act=0, marker=[0,0,0,0]):
        if self.points:
            last_point_height = list(self.points.keys())[-1]
            if last_point_height - h_pos_act > 10:
                self.points[h_pos_act] = marker
        else:
            self.points[h_pos_act] = marker

    def stop_point(self, h_pos_act=0, marker=[0,0,0,0]):
        self.setting_timer.stop()
        self.setting_timer.killTimer(self.setting_timer.timerId())
        self.setting_window.ui.label_hoist_height_bottom.setText(str(h_pos_act))
        self.setting_window.ui.label_xyxy_bottom.setText(str(marker))
        with open('/home/ros/M58_auto/src/gui/param/param.yaml', 'r', encoding='utf-8') as file:
            data = yaml.safe_load(file)
            print(data)
            data['/base/gui_node']['ros__parameters']['marker_ref']=self.points
            print(data)
        with open('/home/ros/M58_auto/src/gui/param/param.yaml', 'w', encoding='utf-8') as file:
            yaml.safe_dump(data, file, sort_keys=False)

    def set_target(self):
        pass

def main():
    rclpy.init()
    app = QApplication(sys.argv)
    app.setStyle('Fusion')
    gui_node = Gui()
    autoapp = AutoAPP(gui_node)
    autoapp.show()
    t = threading.Thread(target=rclpy.spin, args= (gui_node,))
    t.start()
    sys.exit(app.exec_())

