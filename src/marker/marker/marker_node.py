import math
import yaml
import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from sensor_msgs.msg import Image
from interface.msg import Marker
from cv_bridge import CvBridge, CvBridgeError
from interface.msg import PLC



#     0-------------------0
#  ^  |            ___    |
#  |  |t_center . |___|   |
#  op |         g_center  |
#     0-------------------0



class MarkerProcess(Node):
    def __init__(self):
        super().__init__('marker_node')
        self.declare_parameter('length', 900)
        self.declare_parameter('width', 300)
        self.declare_parameter('threshold_bin', 100)
        self.declare_parameter('kernel', 51)
        self.declare_parameter('threshold_k', 1.15)

        self.length = self.get_parameter('length').get_parameter_value().integer_value
        self.width = self.get_parameter('width').get_parameter_value().integer_value
        self.threshold_bin = self.get_parameter('threshold_bin').get_parameter_value().integer_value
        self.kernel = self.get_parameter('kernel').get_parameter_value().integer_value
        self.threshold_k = self.get_parameter('threshold_k').get_parameter_value().double_value

        self.cam_sub = self.create_subscription(Image, 'ipcamera', self.cam_sub_cb, 10)
        self.plc_sub = self.create_subscription(PLC, 'plc', self.plc_sub_cb, 10)
        self.marker_pub = self.create_publisher(Marker, 'marker', 10)
        self.marker_img_pub = self.create_publisher(Image, 'ipcamera_marker', 10)

        self.marker_msg = Marker()
        self.plc_msg = PLC()
        self.cvb = CvBridge()
        self.k = self.length/self.width       
        self.h_pos_act = 0

        with open('/home/ros/M58_auto/src/gui/param/param.yaml', 'r', encoding='utf-8') as file:
            data = yaml.safe_load(file)
            ref_center = data['/base/gui_node']['ros__parameters']['marker_ref']

        center_height = list(map(int, list(ref_center.keys())))
        center_point = list(ref_center.values())
        combined_list = [[x, y] for x, y in zip(center_height, center_point)]
        transformed_list = [[i[0]] + i[1] for i in combined_list]
        data_np = np.array(transformed_list)
        self.combined_np = data_np[data_np[:, 0].argsort()]

    def get_marker_ref(self, hoist_height=0, length=0):
        index_np = np.where(self.combined_np[:, 0] > hoist_height)[0]
        if len(index_np) > 0:
            result_np = self.combined_np[index_np[0]]
            hb_center_set = result_np[1:3]
            g_l = result_np[3]
            distance_scale = length / g_l 
            
        else:
            result_np_min = self.combined_np[np.argmin(self.combined_np[:, 0])]
            result_np_max = self.combined_np[np.argmax(self.combined_np[:, 0])]
            if hoist_height >= result_np_max[0]:
                hb_center_set = result_np_max[1:3]
                distance_scale = length / result_np_max[3]
            elif hoist_height <= result_np_min[0]:
                hb_center_set = result_np_min[1:3] 
                distance_scale = length / result_np_min[3]
            else:
                hb_center_set = [0, 0]
                distance_scale = 0.0
        return distance_scale, hb_center_set

    def cam_sub_cb(self, sensor_img):
        try:
            img = self.cvb.imgmsg_to_cv2(sensor_img)
        except CvBridgeError as e:
            self.get_logger().error("cv bridge error: %s" %e)
        l_area, l_points, l_points_sort = [], [], []
        #预处理
        img_gray = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
        img_gausi = cv2.GaussianBlur(img_gray, (self.kernel,self.kernel), 0)  # 均值滤波
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (self.kernel, self.kernel))
        closing_image = cv2.morphologyEx(img_gausi, cv2.MORPH_CLOSE, kernel)    # 闭运算则是先做膨胀再做腐蚀
        
        # # 膨胀操作可以用来扩大物体边界，常用于填补物体内部的小孔或增加物体的尺寸
        # dilated_image = cv2.dilate(image, kernel)
        # # 腐蚀操作则相反，它会缩小物体的边界，常用于移除边缘上的小点或去除噪声。
        # eroded_image = cv2.erode(image, kernel)
        # # 开运算通常先执行腐蚀再执行膨胀，它有助于消除细小的噪声并保持物体的轮廓清晰。
        # opening_image = cv2.morphologyEx(img_gausi, cv2.MORPH_OPEN, kernel)
        # # 闭运算则是先做膨胀再做腐蚀，这有助于填充小的空洞并将物体合并。
        # # 执行形态学梯度计算（膨胀减去腐蚀）
        # gradient_image = cv2.morphologyEx(closing_image, cv2.MORPH_GRADIENT, kernel)
        # closing_image = cv2.Canny(img_gray, 100, 300)
        
        ret, img_bin = cv2.threshold(closing_image, self.threshold_bin, 255, cv2.THRESH_BINARY)
        contours, hierarchy = cv2.findContours(img_bin, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cv2.drawContours(img, contours, -1, (0,0,255), thickness=3)
        # 计算边界框
        for contour in contours:
            #逼近轮廓
            approx = cv2.approxPolyDP(contour, 0.06*cv2.arcLength(contour, True), True)
            cv2.drawContours(img, [approx], -1, (255,0,0), thickness=3)
            if len(approx) == 4:
                reshape_points = approx.reshape(4,2)
                sorted_points = reshape_points[np.argsort(reshape_points[: , 0])]
                l_p = sorted_points[0:2][np.argsort(sorted_points[0:2][: , 1])]
                r_p = sorted_points[2:4][np.argsort(sorted_points[2:4][: , 1])]
                sl = l_p[0]
                ll = l_p[1]
                sr = r_p[0]
                lr = r_p[1]
                k1 = (sr[0] - sl[0]) / (ll[1] - sl[1])
                k2 = (lr[0] - ll[0]) / (lr[1] - sr[1])
                avg_k = (k1+k2) / 2
                if self.k / self.threshold_k < avg_k < self.k * self.threshold_k:
                    area = cv2.contourArea(contour)
                    l_area.append(area)
                    l_points_sort.append([sl,ll,sr,lr])
                    l_points.append(approx)
        if len(l_points):
            final_points = l_points[l_area.index(max(l_area))]
            final_points_sort = l_points_sort[l_area.index(max(l_area))]
            cv2.drawContours(img, [np.asarray(final_points)], -1, (0,255,0), thickness=5)
            angle1 = np.arctan2(final_points_sort[2][1] - final_points_sort[0][1], final_points_sort[2][0] - final_points_sort[0][0])
            angle2 = np.arctan2(final_points_sort[3][1] - final_points_sort[1][1], final_points_sort[3][0] - final_points_sort[1][0])
            angle = (angle1 + angle2)/2
            angle_degree = math.degrees(angle)
            self.marker_msg.s = round(angle_degree, 2)
            self.marker_msg.g_x = int(((final_points_sort[0][0] + final_points_sort[2][0]) + (final_points_sort[1][0] + final_points_sort[3][0]))/4)
            self.marker_msg.t_y = int(((final_points_sort[0][1] + final_points_sort[1][1]) + (final_points_sort[2][1] + final_points_sort[3][1]))/4)
            self.marker_msg.g_l = int((np.linalg.norm(final_points_sort[2] - final_points_sort[0]) + np.linalg.norm(final_points_sort[3] - final_points_sort[1]))/2) 
            self.marker_msg.t_l = int((np.linalg.norm(final_points_sort[3] - final_points_sort[2]) + np.linalg.norm(final_points_sort[1] - final_points_sort[0]))/2)
            
            self.marker_msg.scale, gx_ty = \
                self.get_marker_ref(self.h_pos_act, self.length)
            self.marker_msg.g_x_ref = int(gx_ty[0])
            self.marker_msg.t_y_ref = int(gx_ty[1])
            self.marker_msg.dis_diff = int((self.marker_msg.t_y - self.marker_msg.t_y_ref) * self.marker_msg.scale)

            self.marker_pub.publish(self.marker_msg)

            # img = cv2.circle(img, (self.marker_msg.g_x_ref, self.marker_msg.t_y_ref), 15, [0, 255, 0], -1)
            cv2.drawMarker(img, (self.marker_msg.g_x_ref, self.marker_msg.t_y_ref), [0, 255, 0], markerType=cv2.MARKER_CROSS, markerSize=35, thickness=5)

        print(self.marker_msg)

        img_pub = self.cvb.cv2_to_imgmsg(img)
        self.marker_img_pub.publish(img_pub)
        # img_resize = cv2.resize(img,(800,600))
        # cv2.imshow('xx', img_resize)
        # cv2.waitKey(1)
    def plc_sub_cb(self, msg:PLC):
        self.h_pos_act = msg.h_pos_act

def main():
    rclpy.init()
    mp = MarkerProcess()
    rclpy.spin(mp)
    mp.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

