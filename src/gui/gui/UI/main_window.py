import sys

from PyQt5.QtGui import QImage, QPixmap
from PyQt5.QtWidgets import QMainWindow, QApplication
from UI import main_ui, setting_window
import pyqtgraph as pg


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.ui = main_ui.Ui_MainWindow()
        self.ui.setupUi(self)

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
        self.ui.widget.layout().addWidget(self.trend_chart_1)
        self.ui.widget_2.layout().addWidget(self.trend_chart_2)
        self.ui.widget_3.layout().addWidget(self.trend_chart_3)

    def show_pix(self, image):
        if image is not None:
            height, width, channel = image.shape
            bytes_per_line = 3 * width
            q_image = QImage(image.data, width, height, bytes_per_line, QImage.Format_RGB888).rgbSwapped()

            # Display the image in the QLabel
            pixmap = QPixmap.fromImage(q_image)
            self.ui.label_img.setPixmap(pixmap)

    def show_chart1(self, data1, data2, data3):
        self.trend_curve1_1.setData(data1)
        self.trend_curve1_2.setData(data2)
        self.trend_curve1_3.setData(data3)

    def show_chart2(self, data1, data2, data3):
        self.trend_curve2_1.setData(data1)
        self.trend_curve2_2.setData(data2)
        self.trend_curve2_3.setData(data3)

    def show_chart3(self, data1, data2):
        self.trend_curve3_1.setData(data1)
        self.trend_curve3_2.setData(data2)

    def show_center_set(self, center_set):
        self.ui.label_hbCenterSet.setText(center_set)

    def show_center_act(self, center_act):
        self.ui.label_hbCenterAct.setText(center_act)

    def show_spd_cmd(self, spd_cmd):
        self.ui.label_trolleySpdCmd.setText(spd_cmd)

    def show_spd_set(self, spd_set):
        self.ui.label_trolleySpdSet.setText(spd_set)

    def show_spd_act(self, spd_act):
        self.ui.label_trolleySpdAct.setText(spd_act)

    def show_skew_act(self, skew_act):
        self.ui.label_hbSkewAct.setText(skew_act)

    def show_skew_set(self, skew_set):
        self.ui.label_hbSkewSet.setText(skew_set)


if __name__ == '__main__':
    app = QApplication(sys.argv)
    app.setStyle('Fusion')
    main_window = MainWindow()
    main_window.show()
    setting_window = setting_window.SettingWindow()
    main_window.ui.actionSettings.triggered.connect(setting_window.show)
    # main_window.ui.menuSettings.triggered.connect(show_setting)
    sys.exit(app.exec_())
