# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'comm_ui.ui'
#
# Created by: PyQt5 UI code generator 5.15.10
#
# WARNING: Any manual changes made to this file will be lost when pyuic5 is
# run again.  Do not edit this file unless you know what you are doing.


from PyQt5 import QtCore, QtGui, QtWidgets


class Ui_CommWindow(object):
    def setupUi(self, CommWindow):
        CommWindow.setObjectName("CommWindow")
        CommWindow.resize(456, 304)
        self.formLayout = QtWidgets.QFormLayout(CommWindow)
        self.formLayout.setObjectName("formLayout")
        self.groupBox = QtWidgets.QGroupBox(CommWindow)
        self.groupBox.setObjectName("groupBox")
        self.gridLayout = QtWidgets.QGridLayout(self.groupBox)
        self.gridLayout.setObjectName("gridLayout")
        self.label_ID_recv = QtWidgets.QLabel(self.groupBox)
        self.label_ID_recv.setObjectName("label_ID_recv")
        self.gridLayout.addWidget(self.label_ID_recv, 0, 1, 1, 1)
        self.label_trolley_position = QtWidgets.QLabel(self.groupBox)
        self.label_trolley_position.setObjectName("label_trolley_position")
        self.gridLayout.addWidget(self.label_trolley_position, 2, 2, 1, 1)
        self.label_trolley_speed_cmd = QtWidgets.QLabel(self.groupBox)
        self.label_trolley_speed_cmd.setObjectName("label_trolley_speed_cmd")
        self.gridLayout.addWidget(self.label_trolley_speed_cmd, 3, 2, 1, 1)
        self.label_5 = QtWidgets.QLabel(self.groupBox)
        self.label_5.setObjectName("label_5")
        self.gridLayout.addWidget(self.label_5, 4, 0, 1, 2)
        self.label_4 = QtWidgets.QLabel(self.groupBox)
        self.label_4.setObjectName("label_4")
        self.gridLayout.addWidget(self.label_4, 3, 0, 1, 2)
        self.label_18 = QtWidgets.QLabel(self.groupBox)
        self.label_18.setObjectName("label_18")
        self.gridLayout.addWidget(self.label_18, 5, 0, 1, 2)
        self.label_3 = QtWidgets.QLabel(self.groupBox)
        self.label_3.setObjectName("label_3")
        self.gridLayout.addWidget(self.label_3, 2, 0, 1, 2)
        self.label = QtWidgets.QLabel(self.groupBox)
        self.label.setObjectName("label")
        self.gridLayout.addWidget(self.label, 0, 0, 1, 1)
        self.label_2 = QtWidgets.QLabel(self.groupBox)
        self.label_2.setObjectName("label_2")
        self.gridLayout.addWidget(self.label_2, 1, 0, 1, 1)
        self.label_hoist_height = QtWidgets.QLabel(self.groupBox)
        self.label_hoist_height.setObjectName("label_hoist_height")
        self.gridLayout.addWidget(self.label_hoist_height, 1, 1, 1, 1)
        self.label_trolley_speed_limit = QtWidgets.QLabel(self.groupBox)
        self.label_trolley_speed_limit.setObjectName("label_trolley_speed_limit")
        self.gridLayout.addWidget(self.label_trolley_speed_limit, 5, 2, 1, 1)
        self.label_trolley_speed_act = QtWidgets.QLabel(self.groupBox)
        self.label_trolley_speed_act.setObjectName("label_trolley_speed_act")
        self.gridLayout.addWidget(self.label_trolley_speed_act, 4, 2, 1, 1)
        self.formLayout.setWidget(0, QtWidgets.QFormLayout.LabelRole, self.groupBox)
        self.groupBox_2 = QtWidgets.QGroupBox(CommWindow)
        self.groupBox_2.setObjectName("groupBox_2")
        self.label_16 = QtWidgets.QLabel(self.groupBox_2)
        self.label_16.setGeometry(QtCore.QRect(10, 30, 81, 16))
        self.label_16.setObjectName("label_16")
        self.label_17 = QtWidgets.QLabel(self.groupBox_2)
        self.label_17.setGeometry(QtCore.QRect(10, 70, 111, 16))
        self.label_17.setObjectName("label_17")
        self.label_ID_send = QtWidgets.QLabel(self.groupBox_2)
        self.label_ID_send.setGeometry(QtCore.QRect(90, 30, 54, 12))
        self.label_ID_send.setObjectName("label_ID_send")
        self.label_speed_set = QtWidgets.QLabel(self.groupBox_2)
        self.label_speed_set.setGeometry(QtCore.QRect(130, 70, 54, 12))
        self.label_speed_set.setObjectName("label_speed_set")
        self.formLayout.setWidget(0, QtWidgets.QFormLayout.FieldRole, self.groupBox_2)

        self.retranslateUi(CommWindow)
        QtCore.QMetaObject.connectSlotsByName(CommWindow)

    def retranslateUi(self, CommWindow):
        _translate = QtCore.QCoreApplication.translate
        CommWindow.setWindowTitle(_translate("CommWindow", "Dialog"))
        self.groupBox.setTitle(_translate("CommWindow", "PLC->PG"))
        self.label_ID_recv.setText(_translate("CommWindow", "1582"))
        self.label_trolley_position.setText(_translate("CommWindow", "2544"))
        self.label_trolley_speed_cmd.setText(_translate("CommWindow", "2544"))
        self.label_5.setText(_translate("CommWindow", "Trolley Speed Act："))
        self.label_4.setText(_translate("CommWindow", "Trolley Speed Cmd："))
        self.label_18.setText(_translate("CommWindow", "Trolley Speed Limit："))
        self.label_3.setText(_translate("CommWindow", "Trolley Position："))
        self.label.setText(_translate("CommWindow", "Telegram ID："))
        self.label_2.setText(_translate("CommWindow", "Hoist Height："))
        self.label_hoist_height.setText(_translate("CommWindow", "8546"))
        self.label_trolley_speed_limit.setText(_translate("CommWindow", "2544"))
        self.label_trolley_speed_act.setText(_translate("CommWindow", "2544"))
        self.groupBox_2.setTitle(_translate("CommWindow", "PG->PLC"))
        self.label_16.setText(_translate("CommWindow", "Telegram ID："))
        self.label_17.setText(_translate("CommWindow", "Trolley Speed Set："))
        self.label_ID_send.setText(_translate("CommWindow", "2544"))
        self.label_speed_set.setText(_translate("CommWindow", "2544"))
