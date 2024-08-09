import json
import time

import numpy as np

from camera import Camera


class SC_Process:
    def __init__(self):

        self.ramp_down = None
        self.sc_done = False
        self.ramp_up = None
        self.min_amplitude_sign = False
        self.max_amplitude_sign = False
        self.duration = 0
        self.max_amplitude = 0
        try:
            with open('./config.json', 'r') as cfg:
                cfg_dit = json.load(cfg)
                ref_center = cfg_dit['process']['points']
                self.start_height = cfg_dit['process']['point_start'][0]
                self.start_xyxy = cfg_dit['process']['point_start'][1]
                self.stop_height = cfg_dit['process']['point_stop'][0]
                self.stop_xyxy = cfg_dit['process']['point_stop'][1]
                self.hb_hor_dimension = cfg_dit['crane_data']['headblock_dimension']['hb_horizontal']
                self.hb_ver_dimension = cfg_dit['crane_data']['headblock_dimension']['hb_vertical']
                self.boom_end_pos = cfg_dit['crane_data']['trolley_position']['boom_end']
                self.ss_sillbeam_pos = cfg_dit['crane_data']['trolley_position']['ss_sillbeam']
                self.sl_sillbeam_pos = cfg_dit['crane_data']['trolley_position']['sl_sillbeam']
                self.ls_sillbeam_pos = cfg_dit['crane_data']['trolley_position']['ls_sillbeam']
                self.ll_sillbeam_pos = cfg_dit['crane_data']['trolley_position']['ll_sillbeam']
                self.backreach_end_pos = cfg_dit['crane_data']['trolley_position']['backreach_end']
                self.up_end_height = cfg_dit['crane_data']['hoist_position']['up_end']
                self.s_sillbeam_height = cfg_dit['crane_data']['hoist_position']['s_sillbeam']
                self.l_sillbeam_height = cfg_dit['crane_data']['hoist_position']['l_sillbeam']
                self.l_down_end_height = cfg_dit['crane_data']['hoist_position']['land_down_end']
                self.s_down_end_height = cfg_dit['crane_data']['hoist_position']['sea_down_end']
                self.trolley_max_spd = cfg_dit['crane_data']['trolley_speed']['max_speed']
                self.trolley_acc_time = cfg_dit['crane_data']['trolley_speed']['acc_time']
                self.trolley_dec_time = cfg_dit['crane_data']['trolley_speed']['dec_time']
                self.hoist_max_spd = cfg_dit['crane_data']['hoist_speed']['max_speed']
                self.hoist_acc_time = cfg_dit['crane_data']['hoist_speed']['acc_time']
                self.hoist_dec_time = cfg_dit['crane_data']['hoist_speed']['dec_time']
                self.pendulum_max = cfg_dit['crane_data']['trolley_height']

                center_height = list(map(int, list(ref_center.keys())))
                center_point = list(ref_center.values())
                combined_list = [[x, y] for x, y in zip(center_height, center_point)]
                transformed_list = [[i[0]] + i[1] for i in combined_list]
                self.combined_np = np.array(transformed_list)

        except FileNotFoundError:
            print("Config file not found.")
            # 如果文件不存在，则可以提供默认值或者抛出异常进行处理

        self.integral = 0
        self.prev_error = 0
        self.speed_interior = 0
        self.set_spd = [0]
        self.smooth_spd = [0]
        self.distance_diff_record = [0]
        self.Kp = 1.0
        self.Ki = 0.001
        self.Kd = 0.001
        url = 'rtsp://admin:hhmc123456@192.168.16.64/Streaming/Channels/2'
        self.camera = Camera(url)

    def sc_main(self, q_put, share_list, trolley_spd_auto, target_trolley):
        # while True:
        t = time.time()
        Kp = share_list[0]
        Ki = share_list[1]
        Kd = share_list[2]
        hoist_height = share_list[5]
        trolley_position = share_list[6]
        # trolley_spd_set = share_list[7]
        # if trolley_spd_set == 0:
        trolley_spd_set = trolley_spd_auto
        trolley_spd_act = share_list[8]
        share_list[3] = self.integral
        share_list[4] = self.prev_error
        distance_scale, hb_center_set = self.get_hb_center(hoist_height)

        frame = self.camera.get_frame()

        if frame is not None:
            img, xyxy = self.camera.process_frame(frame, hb_center_set)
            if len(xyxy) > 0:
                hb_center_act = [int((xyxy[0][2] + xyxy[0][0]) / 2), int((xyxy[0][3] + xyxy[0][1]) / 2)]

                distance_diff = (hb_center_act[1] - hb_center_set[1]) * distance_scale

                dt = time.time() - t

                self.set_pid_constants(Kp, Ki, Kd)
                pid_offset = self.pid(distance_diff, dt)
                duration = self.pendulum_model_duration(self.pendulum_max - hoist_height / 1000)
                duration = duration / 4
                sway = self.find_max_amplitude(distance_diff, dt, duration)
                # print("\ramplitude：", sway, round(duration, 2), end='')

                s_offset_calculate = self.calculate_swing_amplitude((self.pendulum_max - hoist_height / 1000),
                                                                    self.trolley_max_spd)

                s_offset = abs(s_offset_calculate) + abs(sway['max_amplitude'])
                # !!!!!!!Should consider Kp value, the speed adjust rapidly with high Kp
                if abs(trolley_spd_set) > 3:

                    v_cmd = self.speed_after_limit((target_trolley * 1000), trolley_position, trolley_spd_act,
                                                   trolley_spd_set, s_offset)

                    args = {'v_now': trolley_spd_act, 'v_cmd': v_cmd, 'pid_offset': pid_offset,
                            'dt': dt, 'trolley_position': trolley_position, 'ramp_up_time': self.trolley_acc_time,
                            'ramp_down_time': self.trolley_acc_time, 'max_spd_per': 0.9}
                    trolley_spd_cmd = self.speed_with_ramp(**args)
                else:
                    trolley_spd_cmd = trolley_spd_set

                as_require = self.sway_control_done(sway, trolley_spd_act, trolley_spd_set)
                if trolley_spd_auto == 0:
                    share_list[9] = as_require
                else:
                    share_list[9] = 1.0
                share_list[10] = trolley_spd_cmd
                q_dict = {'hoist_height': hoist_height, 'img': img, 'xyxy': xyxy[0], 'center_set': hb_center_set,
                          'center_act': hb_center_act, 'trolley_spd_set': trolley_spd_set, 'as_require': as_require,
                          'trolley_spd_act': trolley_spd_act, 'distance_diff': distance_diff,
                          'trolley_position': trolley_position, 'trolley_spd_cmd': trolley_spd_cmd,
                          'setpoint': pid_offset}

            else:
                q_dict = {'hoist_height': hoist_height, 'img': img, 'xyxy': [], 'center_set': hb_center_set,
                          'center_act': 0, 'trolley_spd_set': trolley_spd_set, 'as_require': 0,
                          'trolley_spd_act': trolley_spd_act, 'distance_diff': 0,
                          'trolley_position': trolley_position, 'trolley_spd_cmd': 0,
                          'setpoint': 0}

            q_put.put(q_dict)

    def sway_control_done(self, sway, trolley_spd_act, trolley_spd_set):
        if int(trolley_spd_set) != 0:
            self.sc_done = False
            as_require = 1.0
        else:
            if sway['max_amplitude'] != 0 and not self.sc_done:
                self.sc_done = False
                as_require = 1.0
            else:
                if abs(trolley_spd_act) < 2:
                    self.sc_done = True
                    as_require = 0.0
                else:
                    self.sc_done = False
                    as_require = 1.0
        return as_require

    # def speed_limit_by_structure(self, s_offset, target, trolley_position, trolley_spd_act, trolley_spd_set):
    #     if trolley_spd_act >= 0:
    #         # target = 60000
    #         speed_limit = self.speed_limit(target, trolley_position, trolley_spd_act, s_offset)
    #         if speed_limit is not None:
    #             v_cmd = min(trolley_spd_set, speed_limit)
    #         else:
    #             v_cmd = trolley_spd_set
    #     else:
    #         # target = 10000
    #         speed_limit = self.speed_limit(target, trolley_position, trolley_spd_act, s_offset)
    #         if speed_limit is not None:
    #             v_cmd = max(trolley_spd_set, speed_limit)
    #         else:
    #             v_cmd = trolley_spd_set
    #     return v_cmd

    def get_hb_center(self, hoist_height):
        index_np = np.where(self.combined_np[:, 0] > hoist_height)[0]
        if len(index_np) > 0:
            result_np = self.combined_np[index_np[-1], 1:].tolist()
            hb_center_set = [int((result_np[2] + result_np[0]) / 2), int((result_np[3] + result_np[1]) / 2)]
            distance_scale = (self.hb_hor_dimension / (result_np[2] - result_np[0]) + self.hb_ver_dimension / (
                    result_np[3] - result_np[1])) / 2
        else:
            hb_center_set = [0, 0]
            distance_scale = 0
        return distance_scale, hb_center_set

    def set_pid_constants(self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd

    def pid(self, distance_diff, dt):
        error = 0.0 - distance_diff
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt
        dis_offset = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
        self.prev_error = error
        return dis_offset

    def speed_with_ramp(self, **kwargs) -> float:
        """
            v_now: speed input
            v_cmd: speed command
            dt: time of speed input
            pid_offset: pid -> predict point - reference point, (-) hb at front (+) hb at back
            ramp_up_time: drive up ramp
            ramp_down_time: drive down ramp
            max_spd_per: percentage 0.0 - 1.0
        :return: speed percentage control for next cycle
        """
        v_now = kwargs.get('v_now', 0.0)
        v_cmd = kwargs.get('v_cmd', 0.0)
        ramp_up_time = kwargs.get('ramp_up_time', 6.0)
        ramp_down_time = kwargs.get('ramp_down_time', 6.0)
        max_spd_per = kwargs.get('max_spd_per', 0.9)
        pid_offset = kwargs.get('pid_offset', 0)
        dt = kwargs.get('dt', 0)
        trolley_position = kwargs.get('trolley_position', 0)

        self.ramp_up = 100 * max_spd_per / ramp_up_time
        self.ramp_down = -100 * max_spd_per / ramp_down_time

        self.ramp_generator(dt, v_cmd)

        if len(self.set_spd) > 100:
            self.set_spd = self.set_spd[-100:]
            self.smooth_spd = self.smooth_spd[-100:]
        #  清空速度记录列表

        speed_out = self.speed_adjust_smooth(dt, pid_offset, ramp_down_time, ramp_up_time)

        cntr_spd = self.single_direction(speed_out, v_cmd)

        return speed_out

    def ramp_generator(self, dt, v_cmd):
        if v_cmd >= 0:
            if v_cmd > self.speed_interior:
                if self.speed_interior < 0:
                    self.speed_interior -= self.ramp_down * dt
                else:
                    self.speed_interior += self.ramp_up * dt
            elif int(v_cmd) == int(self.speed_interior) and (int(v_cmd) != 0 or int(self.speed_interior) == 0):
                self.speed_interior = v_cmd
            else:
                self.speed_interior += self.ramp_down * dt
        else:
            if self.speed_interior > v_cmd:
                if self.speed_interior > 0:
                    self.speed_interior += self.ramp_down * dt
                else:
                    self.speed_interior -= self.ramp_up * dt
            elif int(self.speed_interior) == int(v_cmd) and (int(v_cmd) != 0 or int(self.speed_interior) == 0):
                self.speed_interior = v_cmd
            else:
                self.speed_interior -= self.ramp_down * dt
        #  斜坡后达到控制速度

    def speed_adjust_smooth(self, dt, pid_offset, ramp_down_time, ramp_up_time):
        if dt == 0:
            speed_out = self.speed_interior
            self.set_spd = [0]
        else:
            speed_offset = (pid_offset / dt) / self.trolley_max_spd  # 要补偿调节量的速度!!!!!!!!!!!!!!!!!!!!!!!!!100
            # print("\rspeed adjust： {:.2f}%".format(speed_offset), end='')
            self.set_spd.append(self.speed_interior + speed_offset)  # 基础速度 + 调节速度
            self.smooth_spd = self.moving_average(self.set_spd, 5)

            if len(self.smooth_spd) > 5:
                max_spd_offset = self.smooth_spd[-2] + (100 / ramp_up_time * dt)
                min_spd_offset = self.smooth_spd[-2] - (100 / ramp_down_time * dt)
                speed_out = max(min_spd_offset, min(max_spd_offset, self.smooth_spd[-1]))
                #  限制pid后在基本速度上调整下周期速度变化量????只变一次
            else:
                speed_out = self.speed_interior
        return speed_out

    @staticmethod
    def single_direction(speed_out, v_cmd):
        if v_cmd > 0:
            cntr_spd = max(0, speed_out)
        elif v_cmd < 0:
            cntr_spd = min(0, speed_out)
        else:
            cntr_spd = speed_out
        # 限制最终输出速度, 向前时不可以-速度
        return cntr_spd

    @staticmethod
    def moving_average(data, window_size):
        # 定义一个窗口，其中包含所有权重的平均值
        weights = np.repeat(1.0, window_size) / window_size
        # 使用convolve函数来计算移动平均
        # mode='valid' 表示只计算完全重叠的部分
        return np.convolve(data, weights, 'valid')

    @staticmethod
    def pendulum_model_duration(L):
        # 计算单摆的周期
        period = 2 * np.pi * np.sqrt(L / 9.8)
        return period

    @staticmethod
    def pendulum_model(L, max_amplitude):
        """
        根据摆长和最大摆幅生成单摆的周期，并在整个周期内计算每隔0.01秒的摆角、摆幅、角速度和加速度

        参数:
        L: 单摆长度（米）
        max_amplitude: 最大摆幅（米）

        返回:
        period: 单摆的周期（秒）
        angles: 每隔0.01秒的摆角（弧度）数组
        amplitudes: 每隔0.01秒的摆幅（米）数组
        angular_velocities: 每隔0.01秒的角速度（弧度/秒）数组
        accelerations: 每隔0.01秒的加速度（米/秒^2）数组
        """

        # 如果最大摆幅为0，则将摆角、摆幅、角速度和加速度设置为0
        if max_amplitude == 0:
            period = 0
            angles = np.array([0])
            amplitudes = np.array([0])
            angular_velocities = np.array([0])
            accelerations = np.array([0])
        else:
            # 计算单摆的周期
            period = 2 * np.pi * np.sqrt(L / 9.8)

            # 计算摆角数组
            t = np.arange(0, period, 0.01)
            angles = np.arcsin(max_amplitude / L * np.sin(2 * np.pi / period * t))

            # 计算摆幅数组
            amplitudes = L * np.sin(angles)

            # 计算角速度数组
            angular_velocities = np.ones_like(t) * (2 * np.pi / period)

            # 计算加速度数组
            accelerations = -(9.8 / L) * np.sin(angles)

        return period, angles, amplitudes, angular_velocities, accelerations

    #  initial_angle_radian = np.deg2rad(initial_angle_degree)
    @staticmethod
    def find_closest_value(target_value, array):
        """
        在数组中查找最接近目标值的元素，并返回该元素的索引和值
        eg:在 amplitudes 中查找最接近的值,np.argmin返回最小值的索引
        closest_index, closest_amplitude = find_closest_value(target_amplitude, amplitudes)
        """
        closest_index = np.argmin(np.abs(array - target_value))
        closest_value = array[closest_index]
        return closest_index, closest_value

    def find_max_amplitude(self, distance_diff, dt, duration):
        if abs(distance_diff - self.distance_diff_record[-1]) > 0.05:  # 防摇结束灵敏度
            self.distance_diff_record.append(distance_diff)
            self.duration = 0
        else:
            self.duration += dt
        if len(self.distance_diff_record) > 1:
            ds = self.distance_diff_record[-1] - self.distance_diff_record[-2]
            if distance_diff > 0 > ds / dt and not self.max_amplitude_sign:
                self.max_amplitude = self.distance_diff_record[-2]
                self.distance_diff_record = [0]
                self.max_amplitude_sign = True
                self.min_amplitude_sign = False
            elif distance_diff < 0 < ds / dt and not self.min_amplitude_sign:
                self.max_amplitude = self.distance_diff_record[-2]
                self.distance_diff_record = [0]
                self.min_amplitude_sign = True
                self.max_amplitude_sign = False
        else:
            self.max_amplitude = 0
            self.duration = 0
            self.distance_diff_record = [0]
            self.min_amplitude_sign = False
            self.max_amplitude_sign = False

        if self.duration > duration:
            self.max_amplitude = 0
            self.duration = 0
            self.distance_diff_record = [0]
            self.min_amplitude_sign = False
            self.max_amplitude_sign = False

        return {'max_amplitude': self.max_amplitude}

    def speed_after_limit(self, target_trolley, act_trolley, trolley_spd_act, trolley_spd_set, s_offset):
        if self.ramp_down is None:
            t_dec = self.trolley_dec_time
        else:
            t_dec = abs(100 / self.ramp_down)
        # t_dec = 6
        s = 0.5 * (self.trolley_max_spd / t_dec) * t_dec * t_dec  # 100% / 1/2att
        k = 100 / (s + s_offset)  #

        if trolley_spd_act > 0:
            if target_trolley >= act_trolley:
                trolley_spd_limit = ((target_trolley - act_trolley) / 1000) * k
                trolley_spd_limit = min(100, max(10, trolley_spd_limit))
                v_cmd = min(trolley_spd_set, trolley_spd_limit)
            else:
                v_cmd = trolley_spd_set
        elif trolley_spd_act < 0:
            if target_trolley <= act_trolley:
                trolley_spd_limit = ((target_trolley - act_trolley) / 1000) * k
                trolley_spd_limit = max(-100, min(-10, trolley_spd_limit))
                v_cmd = max(trolley_spd_set, trolley_spd_limit)

            else:
                v_cmd = trolley_spd_set
        else:
            v_cmd = trolley_spd_set

        # print(s, s_offset, trolley_spd_limit)
        return v_cmd

    @staticmethod
    def calculate_swing_amplitude(L, v0):
        # 计算最大振幅
        h = v0 ** 2 / (2 * 9.81)
        a = np.sqrt(L ** 2 - (L - h) ** 2)
        return a
