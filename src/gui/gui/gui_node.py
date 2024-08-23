from itertools import cycle
import threading

from matplotlib import pyplot as plt
from sklearn.cluster import DBSCAN
from UI import main_window, setting_window, alarm_window, comm_window
import numpy as np
from rclpy.node import Node
import rclpy
from interface.msg import Marker, PLC, Trolley, SPSS
import rclpy.time
from sensor_msgs.msg import Image, PointCloud2
import cv_bridge
from PyQt5.QtWidgets import QApplication, QMainWindow
from PyQt5.QtGui import QImage, QPixmap
from PyQt5.QtCore import QTimer
import sys
import cv2
import pyqtgraph as pg
import yaml
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure

import vtk
from vtkmodules.qt.QVTKRenderWindowInteractor import QVTKRenderWindowInteractor
from vtkmodules.util import numpy_support




class Gui(Node):
    def __init__(self):
        super().__init__('gui_node')
        self.sub_marker = self.create_subscription(Marker, 'marker', self.cb_marker, 10)
        self.sub_camera = self.create_subscription(Image, 'ipcamera_marker', self.cb_camera, 10)
        self.sub_plc = self.create_subscription(PLC, 'plc', self.cb_plc, 10)
        self.sub_trolley = self.create_subscription(Trolley, 'trolley', self.cb_trolley, 10)
        self.sub_spss = self.create_subscription(SPSS, 'spss', self.cb_spss, 10)
        self.sub_pc = self.create_subscription(PointCloud2, 'tf_pointcloud2', self.cb_pc, 10)

        self.marker = [0,0,0,0]
        self.t_y = 0
        self.dis_diff = 0
        self.t_y_ref = 0
        self.skew = 0
        self.img = None
        self.profile_fix,self.profile_bay,self.path = [],[],[]
        self.pos = []

    def cb_marker(self, msg:Marker):
        self.marker = [msg.g_x, msg.t_y, msg.g_l, msg.t_l]
        self.t_y = msg.t_y
        self.dis_diff = msg.dis_diff
        self.t_y_ref = msg.t_y_ref
        self.skew = round(msg.s,2)
    
    def cb_camera(self, msg:Image):
        cvb = cv_bridge.CvBridge()
        self.img = cvb.imgmsg_to_cv2(msg)

    def cb_plc(self, msg:PLC):
        self.t_spd_cmd = msg.t_spd_cmd
        self.t_spd_act = msg.t_spd_act
        self.t_spd_set = round(msg.t_spd_set/100, 1)
        self.t_pos_act = msg.t_pos_act
        self.h_spd_cmd = msg.h_spd_cmd
        self.h_spd_act = msg.h_spd_act
        self.h_pos_act = msg.h_pos_act
        self.sc_on_off = msg.sc_on_off
        self.auto_on_off = msg.auto_on_off
        self.t_pos_set = msg.t_pos_set
        self.h_pos_set = msg.h_pos_set

    def cb_trolley(self, msg:Trolley):
        pass

    def cb_spss(self, msg:SPSS):
        self.profile_fix = msg.profile_fix
        self.profile_bay = msg.profile_bay
        self.path = msg.path

    def cb_pc(self, msg:PointCloud2):
        # pc_np = point_cloud2.read_points(msg)
        points = np.frombuffer(msg.data, dtype=np.float32).reshape(-1, 8)
        self.pos =  points[:, :3]



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
        self.data4_1, self.data4_2 = [], []
        self.data_limit = 1000
        self.points = {}


        # 预定义颜色
        self.colors = plt.cm.get_cmap('tab10', 100).colors  # 选择一个足够大的 colormap

        fig = Figure()
        self.canvans = FigureCanvas(fig)
        fig.subplots_adjust(left=0.03, right=0.99, top=1, bottom=0.04)
        self.ax = fig.add_subplot()
        self.ax.tick_params(axis='both',which='major',labelsize=6)
        # self.ax.set_xlim(0, 120)
        # self.ax.set_ylim(-10, 55)
        self.ax.set_xlim(-5, 5)
        self.ax.set_ylim(-5, 5)
        self.line_fix, = self.ax.plot([],[],marker=',', linestyle='-', color='black',markersize=2)
        self.line_block, = self.ax.plot([],[],marker='o', linestyle='-', color='b',markersize=2)
        # self.line_block = self.ax.scatter([],[], marker=',',s = 1)
        self.spd = self.ax.scatter([], [], color='red', marker='s',s = 200)
        self.main_window.ui.tab.layout().addWidget(self.canvans)

        # 初始化 VTK 渲染窗口和交互器
        self.vtkWidget = QVTKRenderWindowInteractor(self.main_window.ui.tab_2)
        self.main_window.ui.tab_2.layout().addWidget(self.vtkWidget)

        # 设置渲染器
        self.ren = vtk.vtkRenderer()
        self.vtkWidget.GetRenderWindow().AddRenderer(self.ren)

        # 初始化点云数据结构
        self.points_vtk = vtk.vtkPoints()
        self.vertices = vtk.vtkCellArray()  # 必需的顶点单元
        self.polydata = vtk.vtkPolyData()
        self.polydata.SetPoints(self.points_vtk)
        self.polydata.SetVerts(self.vertices)  # 将顶点单元添加到 PolyData

        # 设置点云的 Mapper 和 Actor
        mapper = vtk.vtkPolyDataMapper()
        mapper.SetInputData(self.polydata)
        actor = vtk.vtkActor()
        actor.SetMapper(mapper)
        actor.GetProperty().SetColor(1, 0, 0)  # 红色点云
        actor.GetProperty().SetPointSize(3)   # 点大小
        self.ren.AddActor(actor)

        # 设置背景和初始化交互器
        self.ren.SetBackground(0, 0, 0)
        self.vtkWidget.GetRenderWindow().GetInteractor().Initialize()  

        # 手动设置摄像机位置和焦点
        self.ren.GetActiveCamera().SetPosition(3, 1, 10)
        # self.ren.GetActiveCamera().SetFocalPoint(3, 1, 0)
        # self.ren.GetActiveCamera().SetViewUp(0,1,0)
        self.ren.ResetCameraClippingRange()


        self.timer = QTimer()
        self.timer.timeout.connect(lambda: self.show_pix(image=mynode.img))
        self.timer.start(20)

        self.timer1 = QTimer()
        self.timer1.timeout.connect(lambda: self.show_curve(center_act=mynode.t_y, center_set=mynode.t_y_ref, dis_diff=mynode.dis_diff,\
                                                            t_spd_act=mynode.t_spd_act, t_spd_set=mynode.t_spd_set,t_spd_cmd=mynode.t_spd_cmd ))
        self.timer1.timeout.connect(lambda: self.show_data(dis_diff= mynode.dis_diff,skew=mynode.skew, spd_cmd=mynode.t_spd_cmd,\
                                                            spd_set=mynode.t_spd_set, spd_act=mynode.t_spd_act, t_pos_act=mynode.t_pos_act,\
                                                                  t_pos_set=mynode.t_pos_set, h_pos_act=mynode.h_pos_act, h_pos_set=mynode.h_pos_set))
        self.timer1.start(100)

        self.timer2 = QTimer()
        self.timer2.timeout.connect(lambda: self.update_profile(path=mynode.path, profile_fix=mynode.profile_fix, profile_bay=mynode.profile_bay))
        self.timer2.timeout.connect(lambda: self.update_profile_pc(point_array=mynode.pos))
        self.timer2.start(10)

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

        self.trend_chart_4 = pg.PlotWidget()
        self.trend_curve4_1 = self.trend_chart_4.plot(pen='r')  # Blue curve
        self.trend_curve4_2 = self.trend_chart_4.plot(pen='g')  # Blue curve

        # Set the layout for the widget_trace QGroupBox
        self.main_window.ui.widget.layout().addWidget(self.trend_chart_1)
        self.main_window.ui.widget_2.layout().addWidget(self.trend_chart_2)
        self.main_window.ui.widget_3.layout().addWidget(self.trend_chart_3)
        self.main_window.ui.widget_4.layout().addWidget(self.trend_chart_4)

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

    def show_data(self, dis_diff=0, skew=0, spd_cmd=0, spd_set=0, spd_act=0, t_pos_act=0, t_pos_set=0, h_pos_act=0, h_pos_set=0):
        self.main_window.ui.label_disDiff.setText(str(dis_diff))
        self.main_window.ui.label_skew.setText(str(skew))
        self.main_window.ui.label_trolleySpdCmd.setText(str(spd_cmd))
        self.main_window.ui.label_trolleySpdSet.setText(str(spd_set))
        self.main_window.ui.label_trolleySpdAct.setText(str(spd_act))
        self.main_window.ui.label_trolley_act_pos.setText(str(t_pos_act))
        self.main_window.ui.label_trolley_target_pos.setText(str(t_pos_set))
        self.main_window.ui.label_hoist_act_pos.setText(str(h_pos_act))
        self.main_window.ui.label_hoist_target_pos.setText(str(h_pos_set))
        
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
        
        self.data2_1.append(t_spd_set)
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

        self.data4_1.append(h_spd_act)
        self.data4_2.append(h_height)
        self.data4_1 = self.data4_1[-self.data_limit:]
        self.data4_2 = self.data4_2[-self.data_limit:]
        self.trend_curve4_1.setData(self.data4_1)
        self.trend_curve4_2.setData(self.data4_2)

    def closeEvent(self, event):
        rclpy.shutdown()
    def update(self):
        # Update PyQt application
        self.app.processEvents()

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
            yaml.safe_dump(data, file, sort_keys=False, default_flow_style=None)

    def update_profile(self, profile_fix, profile_bay, path):
        profile_fix_array = np.array(profile_fix).reshape(-1,2)
        profile_bay_array = np.array(profile_bay).reshape(-1,2)
        path_array = np.array(path).reshape(-1,2)

        # self.ax.cla()

        # 更新线条数据
        self.line_fix.set_xdata(profile_fix_array[:,0])
        self.line_fix.set_ydata(profile_fix_array[:,1])

        # db = DBSCAN(eps=0.4, min_samples=1).fit(profile_bay_array)
        # labels = db.labels_
        # unique_labels = sorted(set(labels))

        # for label in unique_labels:
        #     cluster_points = profile_bay_array[labels == label]
        #     if label > 99:
        #         color=self.colors[0]
        #     else:
        #         color=self.colors[label]
        #     self.ax.scatter(cluster_points[:, 0], cluster_points[:, 1], s= 5, )

        self.line_block.set_xdata(profile_bay_array[:,0])
        self.line_block.set_ydata(profile_bay_array[:,1])

        # self.line_block.set_offsets(profile_blcok_array)

        self.spd.set_offsets(path_array)

        # 调整X轴范围
        # self.ax.set_xlim(min(self.x_data) - 1, max(self.x_data) + 1)
        # self.ax.set_ylim(min(self.y_data) - 1, max(self.y_data) + 1)

        # 重新绘制图表
        self.canvans.draw()
    
    def update_profile_pc(self, point_array):
        # 清除之前的点云数据
        self.points_vtk.Reset()
        self.vertices.Reset()
        num_points = point_array.shape[0]  # 获取点的数量

        self.points_vtk.SetNumberOfPoints(num_points)
        cells = np.arange(num_points, dtype=np.int64)
        self.vertices.InsertNextCell(num_points, cells)

        # 直接更新 vtkPoints 的数据
        vtk_points_data = numpy_support.numpy_to_vtk(point_array, deep=True)
        self.points_vtk.SetData(vtk_points_data)
            # 通知VTK更新数据
        self.points_vtk.Modified()
        self.vertices.Modified()
        self.polydata.Modified()

        # 刷新渲染器
        self.vtkWidget.GetRenderWindow().Render()
    
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

