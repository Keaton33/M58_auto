'''
import struct
import yaml
import numpy as np
import rclpy
import rclpy.lifecycle
from rclpy.node import Node
from pymodbus.client.tcp import ModbusTcpClient
import rclpy.lifecycle 
from interface.msg import PLC, Trolley, SPSS


class Comm(Node):
    def __init__(self):
        super().__init__("comm_node")
        self.pub_plc = self.create_publisher(PLC, 'plc', 10)
        self.sub_trolley = self.create_subscription(Trolley, 'trolley', self.cb_trolley, 10)
        self.sub_spss = self.create_subscription(SPSS, 'spss', self.cb_spss, 10)

        self.plc = PLC()
        self.id_tmp = 65400
        self.t_spd_cmd,self.t_spd_set,self.t_spd_act,self.t_pos_set,self.t_pos_act,self.h_spd_cmd,\
            self.h_spd_set,self.h_spd_act,self.h_pos_set,self.h_pos_act,self.sc_on_off,self.sc_statue,\
                self.auto_on_off,self.auto_statue,self.id = 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0

        self.declare_parameter("plc_ip", "192.168.1.11")
        self.declare_parameter("plc_port", 503)
        self.declare_parameter("t_spd_cmd", 0) #"a,b,c" a=起始地址 b=数量 c设备地址
        self.declare_parameter("t_spd_set", 1)
        self.declare_parameter("t_spd_act", 2)
        self.declare_parameter("t_pos_set", 3)
        self.declare_parameter("t_pos_act", 5)
        self.declare_parameter("h_spd_cmd", 7)
        self.declare_parameter("h_spd_set", 8)
        self.declare_parameter("h_spd_act", 9)
        self.declare_parameter("h_pos_set", 10)
        self.declare_parameter("h_pos_act", 12)
        self.declare_parameter("sc_on_off", 14)
        self.declare_parameter("sc_statue", 15)
        self.declare_parameter("auto_on_off", 16)
        self.declare_parameter("auto_statue", 17)
        self.declare_parameter("id", 18)

        self.plc_ip = self.get_parameter("plc_ip").get_parameter_value().string_value
        self.plc_port = self.get_parameter("plc_port").get_parameter_value().integer_value
        self.t_spd_cmd_add = self.get_parameter("t_spd_cmd").get_parameter_value().integer_value
        self.t_spd_set_add = self.get_parameter("t_spd_set").get_parameter_value().integer_value
        self.t_spd_act_add = self.get_parameter("t_spd_act").get_parameter_value().integer_value
        self.t_pos_set_add = self.get_parameter("t_pos_set").get_parameter_value().integer_value
        self.t_pos_act_add = self.get_parameter("t_pos_act").get_parameter_value().integer_value
        self.h_spd_cmd_add = self.get_parameter("h_spd_cmd").get_parameter_value().integer_value
        self.h_spd_set_add = self.get_parameter("h_spd_set").get_parameter_value().integer_value
        self.h_spd_act_add = self.get_parameter("h_spd_act").get_parameter_value().integer_value
        self.h_pos_set_add = self.get_parameter("h_pos_set").get_parameter_value().integer_value
        self.h_pos_act_add = self.get_parameter("h_pos_act").get_parameter_value().integer_value
        self.sc_on_off_add = self.get_parameter("sc_on_off").get_parameter_value().integer_value
        self.sc_statue_add = self.get_parameter("sc_statue").get_parameter_value().integer_value
        self.auto_on_off_add = self.get_parameter("auto_on_off").get_parameter_value().integer_value
        self.auto_statue_add = self.get_parameter("auto_statue").get_parameter_value().integer_value
        self.id_add = self.get_parameter("id").get_parameter_value().integer_value
        self.client_ = ModbusTcpClient(self.plc_ip, self.plc_port)
        # self.timer_ = self.create_timer(self.plc_rate/1000, self.cb_timer)

    def cb_trolley(self, msg:Trolley): 
        self.t_spd_set = msg.t_spd_set
        self.plc.t_pos_set = self.t_pos_set
        self.plc.h_spd_set = self.h_spd_set 
        self.plc.h_pos_set = self.h_pos_set
        self.sc_statue = msg.sc_statue
        self.auto_statue = msg.auto_statue
        self.plc.id = self.id
        if not self.client_.connected:
            self.client_.connect()
        if self.client_.connected:
            try:
                self.t_spd_cmd = self.value_read(self.client_.read_holding_registers(self.t_spd_cmd_add).registers[0])
                self.client_.write_register(self.t_spd_set_add, self.value_write(self.t_spd_set))
                self.t_spd_act = self.value_read(self.client_.read_holding_registers(self.t_spd_act_add).registers[0])
                self.client_.write_registers(self.t_pos_set_add, self.dint_value_write(self.t_pos_set))
                self.t_pos_act = self.dint_value_read(self.client_.read_holding_registers(self.t_pos_act_add, 2).registers)
                self.h_spd_cmd = self.value_read(self.client_.read_holding_registers(self.h_spd_cmd_add).registers[0])
                self.client_.write_register(self.h_spd_set_add, self.value_write(self.h_spd_set))
                self.h_spd_act = self.value_read(self.client_.read_holding_registers(self.h_spd_act_add).registers[0])
                self.client_.write_registers(self.h_pos_set_add, self.dint_value_write(self.h_pos_set))
                self.h_pos_act = self.dint_value_read(self.client_.read_holding_registers(self.h_pos_act_add, 2).registers)
                self.sc_on_off = self.value_read(self.client_.read_holding_registers(self.sc_on_off_add).registers[0])
                self.client_.write_register(self.sc_statue_add, self.value_write(self.sc_statue))
                self.auto_on_off = self.value_read(self.client_.read_holding_registers(self.auto_on_off_add).registers[0])
                self.client_.write_register(self.auto_statue_add, self.value_write(self.auto_statue))
                self.client_.write_register(self.id_add, self.value_write(self.id))
            except Exception as e:
                self.get_logger().error("XXXX comm error!!!: %s" %e)
            self.plc.t_spd_cmd = self.t_spd_cmd
            self.plc.t_spd_act = self.t_spd_act
            self.plc.t_spd_set = self.t_spd_set
            self.plc.t_pos_act = self.t_pos_act
            self.plc.h_spd_cmd = self.h_spd_cmd
            self.plc.h_spd_act = self.h_spd_act
            self.plc.h_pos_act = self.h_pos_act
            self.plc.sc_on_off = self.sc_on_off
            self.plc.sc_statue = self.sc_statue
            self.plc.auto_statue = self.auto_statue

            self.plc.auto_on_off = self.auto_on_off
            self.pub_plc.publish(self.plc)
            print(self.plc)
        # self.client_.close()
    
    def cb_spss(self, msg:SPSS):
        self.t_pos_set = msg.target_t
        self.h_pos_set = msg.target_h
    
    def value_write(self, v):
        if v < 0:
            v = (1 << 16) + v # 将负数转换为其 16 位补码表示
        v = v & 0xFFFF     # 确保值在 16 位范围内
        return v

    def dint_value_write(self, v):
        if v < 0:
            v = (1 << 32) + v  # 将负数转换为无符号32位整数的补码
        high = (v >> 16) & 0xFFFF  # 提取高16位
        low = v & 0xFFFF           # 提取低16位
        return [high, low]
    
    def value_read(self, v):
        if v & 0x8000: #检查最高位（第 15 位）是否为 1（即符号位是否为负）。
            v -= 1 << 16 #如果符号位为负，将其转换为负数（即 16 位补码表示）。
        return v

    def dint_value_read(self, v):
        high, low = v[0], v[1]
        value = (high << 16) | low  # 组合两个 16 位寄存器成一个 32 位整数
        if value & 0x80000000:  # 如果最高位（符号位）为 1，将其转换为负数（即 32 位补码表示）
            value -= 1 << 32
        return value

def main():
    rclpy.init()
    comm_node = Comm()
    rclpy.spin(comm_node)
    comm_node.client_.close()
    comm_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

'''

import struct
import rclpy
from rclpy.node import Node
from pymodbus.client.tcp import ModbusTcpClient
from interface.msg import PLC, Trolley, SPSS

class Comm(Node):
    def __init__(self):
        super().__init__("comm_node")
        
        # Initialize publisher and subscribers
        self.pub_plc = self.create_publisher(PLC, 'plc', 10)
        self.sub_trolley = self.create_subscription(Trolley, 'trolley', self.cb_trolley, 10)
        self.sub_spss = self.create_subscription(SPSS, 'spss', self.cb_spss, 10)

        # Initialize PLC message and parameters
        self.plc = PLC()
        self.initialize_parameters()
        self.initialize_modbus_client()

    def initialize_parameters(self):
        # Declare and get parameters with default values
        self.declare_parameter("plc_ip", "192.168.1.11")
        self.declare_parameter("plc_port", 503)

        default_values = {
            "t_spd_cmd": 0, "t_spd_set": 1, "t_spd_act": 2, 
            "t_pos_set": 3, "t_pos_act": 5, "h_spd_cmd": 7, 
            "h_spd_set": 8, "h_spd_act": 9, "h_pos_set": 10, 
            "h_pos_act": 12, "sc_on_off": 14, "sc_statue": 15, 
            "auto_on_off": 16, "auto_statue": 17, "id": 18
        }
        
        for param, default in default_values.items():
            self.declare_parameter(param, default)
            setattr(self, param, self.get_parameter(param).get_parameter_value().integer_value)
            setattr(self, f"{param}_add", self.get_parameter(param).get_parameter_value().integer_value)

        self.plc_ip = self.get_parameter("plc_ip").get_parameter_value().string_value
        self.plc_port = self.get_parameter("plc_port").get_parameter_value().integer_value

    def initialize_modbus_client(self):
        # Initialize Modbus TCP client
        self.client_ = ModbusTcpClient(self.plc_ip, self.plc_port)
        self.connect_modbus_client()

    def connect_modbus_client(self):
        if not self.client_.connected:
            self.client_.connect()
        if not self.client_.connected:
            self.get_logger().error("Failed to connect to PLC.")

    def cb_trolley(self, msg: Trolley):
        # Update internal state from Trolley message
        self.t_spd_set = msg.t_spd_set
        self.plc.t_pos_set = self.t_pos_set
        self.plc.h_spd_set = self.h_spd_set 
        self.plc.h_pos_set = self.h_pos_set
        self.sc_statue = msg.sc_statue
        self.auto_statue = msg.auto_statue
        self.plc.id = self.id

        self.connect_modbus_client()
        if self.client_.connected:
            try:
                # Read and write Modbus registers
                self.perform_modbus_operations()
                # Publish updated PLC message
                self.pub_plc.publish(self.plc)
                self.get_logger().info(str(self.plc))
            except Exception as e:
                self.get_logger().error(f"Modbus communication error: {e}")

    def perform_modbus_operations(self):
        self.t_spd_cmd = self.value_read(self.client_.read_holding_registers(self.t_spd_cmd_add).registers[0])
        self.client_.write_register(self.t_spd_set_add, self.value_write(self.t_spd_set))
        self.t_spd_act = self.value_read(self.client_.read_holding_registers(self.t_spd_act_add).registers[0])
        self.client_.write_registers(self.t_pos_set_add, self.dint_value_write(self.t_pos_set))
        self.t_pos_act = self.dint_value_read(self.client_.read_holding_registers(self.t_pos_act_add, 2).registers)
        
        self.h_spd_cmd = self.value_read(self.client_.read_holding_registers(self.h_spd_cmd_add).registers[0])
        self.client_.write_register(self.h_spd_set_add, self.value_write(self.h_spd_set))
        self.h_spd_act = self.value_read(self.client_.read_holding_registers(self.h_spd_act_add).registers[0])
        self.client_.write_registers(self.h_pos_set_add, self.dint_value_write(self.h_pos_set))
        self.h_pos_act = self.dint_value_read(self.client_.read_holding_registers(self.h_pos_act_add, 2).registers)
        
        self.sc_on_off = self.value_read(self.client_.read_holding_registers(self.sc_on_off_add).registers[0])
        self.client_.write_register(self.sc_statue_add, self.value_write(self.sc_statue))
        self.auto_on_off = self.value_read(self.client_.read_holding_registers(self.auto_on_off_add).registers[0])
        self.client_.write_register(self.auto_statue_add, self.value_write(self.auto_statue))
        self.client_.write_register(self.id_add, self.value_write(self.id))

        # Update PLC message attributes
        self.update_plc_message()

    def update_plc_message(self):
        self.plc.t_spd_cmd = self.t_spd_cmd
        self.plc.t_spd_act = self.t_spd_act
        self.plc.t_spd_set = self.t_spd_set
        self.plc.t_pos_act = self.t_pos_act
        self.plc.h_spd_cmd = self.h_spd_cmd
        self.plc.h_spd_act = self.h_spd_act
        self.plc.h_spd_set = self.h_spd_set
        self.plc.h_pos_act = self.h_pos_act
        self.plc.sc_on_off = self.sc_on_off
        self.plc.sc_statue = self.sc_statue
        self.plc.auto_statue = self.auto_statue
        self.plc.auto_on_off = self.auto_on_off

    def cb_spss(self, msg: SPSS):
        self.t_pos_set = msg.target_t
        self.h_pos_set = msg.target_h

    def value_write(self, v):
        v = (v & 0xFFFF) if v >= 0 else (1 << 16) + v
        return v

    def dint_value_write(self, v):
        v = (1 << 32) + v if v < 0 else v
        return [(v >> 16) & 0xFFFF, v & 0xFFFF]

    def value_read(self, v):
        return v - (1 << 16) if v & 0x8000 else v

    def dint_value_read(self, v):
        value = (v[0] << 16) | v[1]
        return value - (1 << 32) if value & 0x80000000 else value

def main():
    rclpy.init()
    comm_node = Comm()
    rclpy.spin(comm_node)
    comm_node.client_.close()
    comm_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
