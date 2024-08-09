import numpy as np
import rclpy
from rclpy.node import Node
from interface.msg import Marker,PLC,SPSS,Trolley


class TrolleyCtl(Node):
    def __init__(self):
        super().__init__('trolley_control_node')
        self.sub_marker = self.create_subscription(Marker, 'marker', self.cb_marker, 10)
        self.sub_plc = self.create_subscription(PLC, 'plc', self.cb_plc, 10)
        self.sub_spss = self.create_subscription(SPSS, 'spss', self.cb_spss, 10) 
        self.pub_tro = self.create_publisher(Trolley, 'trolley', 10)
        self.trolley = Trolley()

        self.sc_statue = 0
        self.duration_temp = 0
        self.max_amplitude = 0
        self.speed_interior = 0
        self.set_spd = [0]
        self.smooth_spd = [0]
        self.distance_diff_record = [0]

        self.declare_parameter("trolley_spd_b", 3.5)
        self.declare_parameter("trolley_spd_j", 0.0)
        self.declare_parameter("trolley_spd_w", 0.01)
        self.declare_parameter("trolley_rate", 0.05)     #s
        self.declare_parameter("hoist_height", 50000)   #mm
        self.declare_parameter("trolley_spd", 3000)     #mm/s
        self.declare_parameter("trolley_acc", 6.0)      #s
        self.declare_parameter("trolley_dec", 6.0)      #s
        self.declare_parameter("trolley_spd_per", 0.9)  #%
        self.declare_parameter("sway_threshold", 300)   #mm

        self.t_spd_Kp = self.get_parameter("trolley_spd_b").get_parameter_value().double_value
        self.t_spd_Ki = self.get_parameter("trolley_spd_j").get_parameter_value().double_value
        self.t_spd_Kd = self.get_parameter("trolley_spd_w").get_parameter_value().double_value
        self.t_rate = self.get_parameter("trolley_rate").get_parameter_value().double_value
        self.h_pos_max = self.get_parameter("hoist_height").get_parameter_value().integer_value
        self.t_spd_max = self.get_parameter("trolley_spd").get_parameter_value().integer_value
        self.t_acc = self.get_parameter("trolley_acc").get_parameter_value().double_value
        self.t_dec = self.get_parameter("trolley_dec").get_parameter_value().double_value
        self.t_spd_per = self.get_parameter("trolley_spd_per").get_parameter_value().double_value
        self.sc_threshold = self.get_parameter("sway_threshold").get_parameter_value().integer_value

        self.t_spd_pid = PIDController(self.t_spd_Kp, self.t_spd_Ki, self.t_spd_Kd, 0)
        self.t_acc_dt = 100 * self.t_spd_per / self.t_acc   #0-100 rate
        self.t_dec_dt = -100 * self.t_spd_per / self.t_dec
        
        self.g_x,self.t_y,self.skew,self.t_y_ref,self.g_x_ref,self.g_l,self.t_l,self.scale,self.dis_diff=\
        0,0,0,0,0,0,0,0,0
        self.t_spd_cmd,self.t_spd_act,self.t_pos_act,self.h_spd_cmd,self.h_spd_act,self.h_pos_act,self.sc_on_off,self.auto_on_off=\
        0,0,0,0,0,0,0,0
        self.target_trolley = 0

        self.timer_ = self.create_timer(self.t_rate, self.cb_timer)

    def cb_marker(self, msg:Marker):
        self.g_x = msg.g_x
        self.t_y = msg.t_y
        self.skew = msg.s
        self.t_y_ref = msg.t_y_ref
        self.g_x_ref = msg.g_x_ref
        self.g_l = msg.g_l
        self.t_l = msg.t_l
        self.scale = msg.scale
        self.dis_diff = msg.dis_diff
    
    def cb_spss(self, msg:SPSS):
        self.target_trolley = msg.target_t

    def cb_plc(self, msg:PLC):
        self.t_spd_cmd = msg.t_spd_cmd
        self.t_spd_act = msg.t_spd_act
        self.t_pos_act = msg.t_pos_act
        self.h_spd_cmd = msg.h_spd_cmd
        self.h_spd_act = msg.h_spd_act
        self.h_pos_act = msg.h_pos_act
        self.sc_on_off = msg.sc_on_off
        self.auto_on_off = msg.auto_on_off

    def cb_timer(self):
        pid_offset = self.t_spd_pid.update(self.dis_diff, self.t_rate)       #speed control after pid controller
        duration = self.pendulum_model_duration(self.h_pos_max - self.h_pos_act)    #pendulum duration
        self.find_max_amplitude(self.dis_diff, duration, self.sc_threshold, self.t_spd_cmd)    #max sway distance

        s_offset_calculate = self.calculate_swing_amplitude((self.h_pos_max - self.h_pos_act),self.t_spd_max)   #if sudden stop swing amplitude(max_spd->act_spd)
        s_offset = s_offset_calculate + self.max_amplitude     #safety clearence

        if self.sc_on_off == 1:
            # print('target:',self.target_trolley, 'pos:',self.t_pos_act, 'spd_act:',self.t_spd_act, 'spd_cmd:',self.t_spd_cmd, s_offset)
            v_cmd = self.speed_after_limit(self.target_trolley, self.t_pos_act, self.t_spd_cmd, s_offset)
            args = {'v_now': self.t_spd_act, 'v_cmd': v_cmd, 'pid_offset': pid_offset}
            trolley_spd_cmd = self.speed_with_ramp(**args)
        else:
            trolley_spd_cmd = self.t_spd_cmd

        self.trolley.t_spd_set = int(trolley_spd_cmd * 100)
        self.trolley.sc_statue = self.sc_statue
        
        self.pub_tro.publish(self.trolley)

    @staticmethod
    def pendulum_model_duration(L):
        # 计算单摆的周期
        period = 2 * np.pi * np.sqrt(L / 9800)
        return period
    @staticmethod
    def calculate_swing_amplitude(L, v0):
        # 计算最大振幅
        h = v0 ** 2 / (2 * 9800)
        # a = np.sqrt(L ** 2 - (L - h) ** 2)
        a = np.sqrt(L ** 2 - (L - h) ** 2) if (L - h) ** 2 <= L ** 2 else 0
        return a

    def find_max_amplitude(self, distance_diff, duration, threshold, t_spd_cmd):
        self.distance_diff_record.append(abs(distance_diff))
        self.duration_temp += self.t_rate
        if self.duration_temp > duration/4:
            if self.max_amplitude < threshold and t_spd_cmd == 0:
                self.sc_statue = 1

        if self.duration_temp > duration/2:
            self.max_amplitude = np.max(np.array(self.distance_diff_record))
            self.duration_temp = 0
            self.distance_diff_record = [0]

        if distance_diff > threshold or t_spd_cmd != 0:
            self.sc_statue = 0
        # print('recod:',len(self.distance_diff_record), 'max_A:',self.max_amplitude, 'sc_done:',self.sc_statue)
    
    def speed_after_limit(self, target_trolley, act_trolley, trolley_spd_cmd, s_offset):
        if self.t_dec_dt is None:
            t_dec = self.t_dec
        else:
            t_dec = abs(100 / self.t_dec_dt)
        # t_dec = 6
        s = 0.5 * (self.t_spd_max / t_dec) * t_dec * t_dec  # 100% / 1/2att
        k = 100 / (s + s_offset)  #
        if trolley_spd_cmd > 0:
            if target_trolley >= act_trolley:
                trolley_spd_limit = (target_trolley - act_trolley) * k
                trolley_spd_limit = min(100, max(10, trolley_spd_limit))
                v_cmd = min(trolley_spd_cmd, trolley_spd_limit)
            else:
                v_cmd = 0
                # v_cmd = trolley_spd_cmd
        elif trolley_spd_cmd < 0:
            if target_trolley <= act_trolley:
                trolley_spd_limit = (target_trolley - act_trolley) * k
                trolley_spd_limit = max(-100, min(-10, trolley_spd_limit))
                v_cmd = max(trolley_spd_cmd, trolley_spd_limit)

            else:
                v_cmd = 0
                # v_cmd = trolley_spd_cmd
        else:
            v_cmd = trolley_spd_cmd
        return v_cmd
    
    def speed_with_ramp(self, **kwargs) -> float:
        """
            v_now: speed input
            v_cmd: speed command
            dt: time of speed input
            pid_offset: pid -> predict point - reference point, (-) hb at front (+) hb at back
        :return: speed percentage control for next cycle
        """
        v_now = kwargs.get('v_now', 0.0)
        v_cmd = kwargs.get('v_cmd', 0.0)
        pid_offset = kwargs.get('pid_offset', 0)

        self.ramp_generator(v_cmd, v_now)

        if len(self.set_spd) > 50:
            self.set_spd = self.set_spd[-50:]
            self.smooth_spd = self.smooth_spd[-50:]
        #  清空速度记录列表

        speed_out = self.speed_adjust_smooth(pid_offset)

        cntr_spd = self.single_direction(speed_out, v_cmd)

        return speed_out

    def ramp_generator(self, v_cmd, v_now):
        epsilon = max(self.t_dec_dt * self.t_rate, self.t_acc_dt * self.t_rate) # 允许的误差范围，防止抖动

        if v_cmd >= 0:
            if (v_cmd > v_now > self.speed_interior) or (v_cmd < v_now < self.speed_interior):
                self.speed_interior = v_now

            if abs(v_cmd - self.speed_interior) > epsilon:
                if v_cmd > self.speed_interior:
                    self.speed_interior += (self.t_acc_dt if self.speed_interior >= 0 else -self.t_dec_dt) * self.t_rate
                else:
                    self.speed_interior += self.t_dec_dt * self.t_rate
        else:
            if (v_cmd < v_now < self.speed_interior) or (v_cmd > v_now > self.speed_interior):
                self.speed_interior = v_now

            if abs(self.speed_interior - v_cmd) > epsilon:
                if self.speed_interior > v_cmd:
                    self.speed_interior += (-self.t_acc_dt if self.speed_interior <= 0 else self.t_dec_dt) * self.t_rate
                else:
                    self.speed_interior -= self.t_dec_dt * self.t_rate

    def speed_adjust_smooth(self, pid_offset):
        speed_offset = (pid_offset / self.t_rate) / self.t_spd_max  # 要补偿调节量的速度!!!!!!!!!!!!!!!!!!!!!!!!!100
        self.set_spd.append(self.speed_interior + speed_offset)  # 基础速度 + 调节速度
        self.smooth_spd = self.moving_average(self.set_spd, 5)

        if len(self.smooth_spd) > 5:
            max_spd_offset = self.smooth_spd[-2] + (100 / self.t_acc * self.t_rate)
            min_spd_offset = self.smooth_spd[-2] - (100 / self.t_dec * self.t_rate)
            speed_out = max(min_spd_offset, min(max_spd_offset, self.smooth_spd[-1]))
            #  限制pid后在基本速度上调整下周期速度变化量????只变一次
        else:
            speed_out = self.smooth_spd[-1]
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
    
class PIDController:
    def __init__(self, Kp, Ki, Kd, setpoint):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.setpoint = setpoint
        self.integral = 0.0
        self.previous_error = 0.0

    def update(self, current_value, sample_time):
        error = self.setpoint - current_value
        self.integral += error * sample_time
        derivative = (error - self.previous_error) / sample_time
        output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
        self.previous_error = error
        return output

def main():
    rclpy.init()
    trolleyctl = TrolleyCtl()
    rclpy.spin(trolleyctl)
    trolleyctl.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
