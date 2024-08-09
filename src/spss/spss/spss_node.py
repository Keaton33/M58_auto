import yaml
import numpy as np
import rclpy
from rclpy.node import Node
from interface.msg import SPSS, PLC
import time

class Profile(Node):
    def __init__(self):
        super().__init__('spss_node')
        self.pub_spss = self.create_publisher(SPSS, 'spss', 10)
        self.sub_plc = self.create_subscription(PLC, 'plc', self.cb_plc, 10)
        self.spss = SPSS()

        '''
        with open('/home/ros/M58_auto/src/gui/param/param.yaml', 'r', encoding='utf-8') as file:
            data = yaml.safe_load(file)
            print(data)
            data['/base/spss_node']['ros__parameters']['fixed_obstacle']=[[0, 200, 9000], [9000, 8000, 12000],[12000,200,43000],[43000,8000,46000],[46000,0,90000]]
            print(data)
        with open('/home/ros/M58_auto/src/gui/param/param.yaml', 'w', encoding='utf-8') as file:
            yaml.safe_dump(data, file, sort_keys=False)
        '''
        
        with open('/home/ros/M58_auto/src/gui/param/param.yaml', 'r', encoding='utf-8') as file:
            data = yaml.safe_load(file)
            fixed_obstacle = data['/base/spss_node']['ros__parameters']['fixed_obstacle']
            data_np = np.array(fixed_obstacle)
            self.fixed_obstacle = data_np[data_np[:, 0].argsort()]
            print(self.fixed_obstacle)
                
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
        self.pub_spss.publish(self.spss)

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
