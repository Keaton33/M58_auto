import ast
import struct
from sklearn.cluster import DBSCAN
import yaml
import numpy as np
import rclpy
from rclpy.node import Node
from interface.msg import SPSS, PLC
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
import time

class Profile(Node):
    def __init__(self):
        super().__init__('spss_node')
        self.pub_spss = self.create_publisher(SPSS, 'spss', 10)
        self.sub_plc = self.create_subscription(PLC, 'plc', self.cb_plc, 10)
        self.sub_pc = self.create_subscription(PointCloud2, 'tf_pointcloud2', self.cb_pc, 10)
        self.spss = SPSS()

        '''
        with open('/home/ros/M58_auto/src/gui/param/param.yaml', 'r', encoding='utf-8') as file:
            data = yaml.safe_load(file)
            print(data)
            profile = [[0, 200, 9000], [9000, 8000, 12000],[12000,200,43000],[43000,8000,46000],[46000,0,90000]]
            data['/base/spss_node']['ros__parameters']['fixed_obstacle']= str(profile)
            print(data)
        with open('/home/ros/M58_auto/src/gui/param/param.yaml', 'w', encoding='utf-8') as file:
            yaml.safe_dump(data, file, sort_keys=False, default_flow_style=None)
        '''
        
        with open('/home/ros/M58_auto/src/gui/param/param.yaml', 'r', encoding='utf-8') as file:
            data = yaml.safe_load(file)
            fixed_obstacle = data['/base/spss_node']['ros__parameters']['fixed_obstacle']
            fixed_obstacle_list = ast.literal_eval(fixed_obstacle)
            data_np = np.array(fixed_obstacle_list)
            self.fixed_obstacle = data_np[data_np[:, 0].argsort()]
        
        fix_profile_data = np.array([
                                [row[0]/1000, row[1]/1000] if i % 2 == 0 else [row[2]/1000, row[1]/1000]
                                for row in self.fixed_obstacle
                                for i in range(2)
                            ])

        self.spss.profile_fix = fix_profile_data.flatten().tolist()[:-2]
   
        self.profile = self.fixed_obstacle  #需结合轮廓扫描！！！
        self.preset_point = [15000, 16000], [30000, 16000], [40000, 12000]  #需根据TOS作业位置及轮廓计算预设点！！！

    def cb_plc(self, msg:PLC):
        self.t_spd_cmd = msg.t_spd_cmd
        self.t_spd_act = msg.t_spd_act
        self.t_pos_act = msg.t_pos_act
        self.h_spd_cmd = msg.h_spd_cmd
        self.h_spd_act = msg.h_spd_act
        self.h_pos_act = msg.h_pos_act
        self.sc_on_off = msg.sc_on_off
        self.auto_on_off = msg.auto_on_off 

        if self.sc_on_off == 1 and self.auto_on_off == 0:
            self.target_t = self.sc_trolley_target()
            self.target_h = self.h_pos_act
        elif self.sc_on_off == 1 and self.auto_on_off == 1:
            [self.target_t, self.target_h] = self.auto_target(self.t_pos_act, self.h_pos_act)
        else:
            self.target_t = 0
            self.target_h = 0

        self.spss.target_t = self.target_t
        self.spss.target_h = self.target_h
        self.spss.path = [self.t_pos_act/1000, self.h_pos_act/1000]
        self.pub_spss.publish(self.spss)

    def cb_pc(self, msg:PointCloud2):
        pc_np = point_cloud2.read_points(msg)
        # pc_np_2d = pc_np.view(np.float32).reshape(pc_np.shape[0], -1)[:,:3]        
        pc_np['x'] = np.round(pc_np['x']/0.05) *0.05
        # 对新的数组按 x 排序
        xy_sorted = np.sort(pc_np, order=['x', 'y'])
        unique_x, indices = np.unique(xy_sorted['x'], return_index=True)
        max_indices = np.minimum.reduceat(np.arange(len(xy_sorted)), indices)
        pc_profile = xy_sorted[max_indices]
        # self.pc_profile = np.column_stack((pc_profile['x'], pc_profile['y']))

        # self.spss.profile_x = pc_profile['x'].tolist()
        # self.spss.profile_y = pc_profile['y'].tolist()
        print(pc_profile)
        self.spss.profile_bay = [55.0,10.0,58.0,10.0,58.0,25.0,61.0,25.0,61.0,-8.0,66.0,-8.0]##################################################################################
        points_bay = np.frombuffer(pc_profile, dtype=np.float32).reshape(-1, 8)[:, :2]

        db = DBSCAN(eps=0.3, min_samples=5).fit(points_bay)
        labels = db.labels_
        # 计算差分，找到变化的点
        diff = np.diff(labels)
        # 计算分组的开始点
        split_points = np.where(diff != 0)[0] + 1
        # 在原始数组上插入切割点的前后
        grouped_indices = np.split(np.arange(labels.size), split_points)
        grouped_points = [points_bay[group].tolist() for group in grouped_indices]
        simplified_groups = []
        for group_indices in grouped_indices:
            group_points = points_bay[group_indices]
            x_min, y_min = group_points.min(axis=0)
            x_max, y_max = group_points.max(axis=0)
            simplified_groups.append([[x_min, y_max], [x_max, y_max]])
        simplified_groups_np = np.array(simplified_groups)
        self.get_logger().info(str(simplified_groups))


        # self.spss.profile_bay = points_bay.flatten().tolist()
        self.spss.profile_bay = simplified_groups_np.flatten().tolist()
        # 打开文件（如果文件不存在，会自动创建）
        with open("example.txt", "w") as file:
            # 写入文本到文件
            for point in points_bay:
                file.write(f"{point[0]:.6f}, {point[1]:.6f}\n")


        pc_marker = (pc_np['x'] > 0) & (0 < pc_np['y']) & (pc_np['y'] < 2) & (-1 < pc_np['z']) & (pc_np['z'] < 1)
        # print(np.min(pc_np[pc_marker]['y']))

    def sc_trolley_target(self):
        if self.t_spd_cmd >= 0:
            a = self.fixed_obstacle[self.fixed_obstacle[:, 0] >= self.t_pos_act]
            a = a[a[:,1] > self.h_pos_act]
            if np.size(a) > 0:
                target_t = int(a[0][0])
            else:
                target_t = int(self.fixed_obstacle[-1][2])
        else:
            a = self.fixed_obstacle[self.fixed_obstacle[:, 2] <= self.t_pos_act]
            a = a[a[:,1] > self.h_pos_act]
            if np.size(a) > 0:
                target_t = int(a[-1][2])
            else:
                target_t = int(self.fixed_obstacle[0][0])
        return target_t

    def auto_target(self, trolley_pos, hoist_pos):
        if self.profile[-1][2] >= self.preset_point[-1][0] >= self.profile[0][0] and self.profile[-1][2] >= self.preset_point[0][0] >= self.profile[0][0]:  # 最终目标位置在轮廓内
            if self.preset_point[-1][0] > trolley_pos and self.preset_point[-1][0] > self.preset_point[0][0]:  # 向前
                for i in self.preset_point:  # 找预设点
                    if i[0] > trolley_pos:  # [15, 16], [30, 16], [40, 12]
                        target_point = i

                        target_trolley = self.preset_point[-1][0]  # 小车目标为最终位置
                        target_hoist = target_point[1]

                        indices = np.where(self.profile[:, 2] > trolley_pos)[0]
                        if np.size(indices) > 1:
                            profile_region_current = self.profile[indices[0]]
                            profile_region_next = self.profile[indices[1]]  # 确保有当前轮廓和下一个轮廓
                        elif np.size(indices) == 1:
                            profile_region_current = self.profile[indices[0]]
                            profile_region_next = profile_region_current  # 确保有当前轮廓和下一个轮廓
                        else:
                            profile_region_current = self.profile[-1]
                            profile_region_next = profile_region_current  # 确保有当前轮廓和下一个轮廓

                        f_down_profile = self.profile[indices]
                        f_down_profile_max_height = np.max(f_down_profile[:, 1])    #没加箱子高度和安全高度

                        profile_region_current_trolley = profile_region_current[2]
                        profile_region_current_hoist = profile_region_current[1]
                        profile_region_next_hoist = profile_region_next[1]
                        # profile_region_next_trolley = profile_region_next[0]

                        if hoist_pos < target_hoist:  # 向上
                            if target_hoist < f_down_profile_max_height:
                                target_hoist = f_down_profile_max_height

                            if hoist_pos > profile_region_next_hoist:  # 增加高度确保安全，hoist_pos要加箱子高度
                                target_next = [target_trolley, target_hoist]
                            elif hoist_pos < profile_region_current_hoist:
                                target_next = [trolley_pos, target_hoist]
                            else:
                                target_next = [profile_region_current_trolley, target_hoist]  # 减小距离确保安全
                        else:  # 向下

                            if hoist_pos > profile_region_next_hoist:  # 增加高度确保安全
                                target_next = [target_trolley, target_hoist]
                            elif hoist_pos < profile_region_current_hoist:
                                target_next = [trolley_pos, hoist_pos]
                            else:
                                if target_trolley > profile_region_current_trolley:
                                    target_next = [profile_region_current_trolley, target_hoist]
                                else:
                                    target_next = [target_trolley, target_hoist]
                        break

            elif self.preset_point[0][0] < trolley_pos and self.preset_point[-1][0] < self.preset_point[0][0]:  # 向后
                for i in self.preset_point:

                    if i[0] < trolley_pos:  # [50, 9], [30, 16], [15, 16]
                        target_point = i

                        target_trolley = self.preset_point[-1][0]
                        target_hoist = target_point[1]

                        indices = np.where(self.profile[:, 0] < trolley_pos)[0]
                        if np.size(indices) > 1:
                            profile_region_current = self.profile[indices[-1]]
                            profile_region_next = self.profile[indices[-2]]  # 确保有当前轮廓和下一个轮廓
                        elif np.size(indices) == 1:
                            profile_region_current = self.profile[indices[-1]]
                            profile_region_next = profile_region_current  # 确保有当前轮廓和下一个轮廓
                        else:
                            profile_region_current = self.profile[0]
                            profile_region_next = profile_region_current  # 确保有当前轮廓和下一个轮廓

                        b_down_profile = self.profile[indices]
                        b_down_profile_max_height = np.max(b_down_profile[:, 1])

                        profile_region_current_trolley = profile_region_current[0]
                        profile_region_current_hoist = profile_region_current[1]
                        profile_region_next_hoist = profile_region_next[1]

                        if hoist_pos < target_hoist:  # 向上
                            if target_hoist < b_down_profile_max_height:
                                target_hoist = b_down_profile_max_height

                            if hoist_pos > profile_region_next_hoist:  # 增加高度确保安全
                                target_next = [target_trolley, target_hoist]
                            elif hoist_pos < profile_region_current_hoist:
                                target_next = [trolley_pos, target_hoist]
                            else:
                                target_next = [profile_region_current_trolley, target_hoist]  # 减小距离确保安全
                        else:  # 向下

                            if hoist_pos > profile_region_next_hoist:  # 增加高度确保安全
                                target_next = [target_trolley, target_hoist]
                            elif hoist_pos < profile_region_current_hoist:
                                target_next = [trolley_pos, hoist_pos]
                            else:
                                if target_trolley < profile_region_current_trolley:
                                    target_next = [profile_region_current_trolley, target_hoist]
                                else:
                                    target_next = [target_trolley, target_hoist]

                        break
            else:
                target_next = [trolley_pos, hoist_pos]
        else:
            target_next = [trolley_pos, hoist_pos]
        return target_next

    def get_profile(self):
        pass

    def set_preset_points(self):
        pass

def main():
    rclpy.init()
    profile = Profile()
    rclpy.spin(profile)
    profile.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
