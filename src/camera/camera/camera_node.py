import ffmpeg
import numpy as np
import cv2
import rclpy
import rclpy.logging
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


class IPCmaeraPublisher(Node):
    def __init__(self):
        super().__init__('camera_node')

        self.declare_parameter('camera_ip', "192.168.1.108" )
        self.declare_parameter('camera_login_user', "admin")
        self.declare_parameter('camera_login_pwd', "DAHUA123")
        self.declare_parameter('camera_channel', 0)

        self.camera_ip = self.get_parameter('camera_ip').get_parameter_value().string_value    # 摄像头ip
        self.camera_login_user = self.get_parameter('camera_login_user').get_parameter_value().string_value
        self.camera_login_pwd = self.get_parameter('camera_login_pwd').get_parameter_value().string_value
        self.camera_channel = self.get_parameter("camera_channel").get_parameter_value().integer_value      # 选择主码流，还是辅码流

        self.alhua_rtsp = f"rtsp://{self.camera_login_user}:{self.camera_login_pwd}@{self.camera_ip}/cam/realmonitor?channel=1&subtype={self.camera_channel}"
        self.args = {
            "rtsp_transport": "tcp",
            "fflags": "nobuffer",
            "flags": "low_delay"
        }    # 添加参数


        self.publisher_ = self.create_publisher(Image, 'ipcamera', 10)
        self.pub_img()

    def pub_img(self):
        self.get_logger().info("Connecting to IPcam..."+ self.alhua_rtsp)
        try:
            self.probe = ffmpeg.probe(self.alhua_rtsp)
        except Exception as e:
            self.probe = None
        if self.probe:
            self.get_logger().info("connected to IPcam:"+ self.probe['format']["filename"])
            self.cap_info = next(x for x in self.probe['streams'] if x['codec_type'] == 'video')
            # print("fps: {}".format(self.cap_info['r_frame_rate']))
            self.width = self.cap_info['width']           # 获取视频流的宽度
            self.height = self.cap_info['height']         # 获取视频流的高度
            # self.up, self.down = str(self.cap_info['r_frame_rate']).split('/')
            # self.fps = eval(self.up) / eval(self.down)
            # print("fps: {}".format(self.fps))    # 读取可能会出错错误
            self.process1 = (
                ffmpeg
                .input(self.alhua_rtsp, **self.args)
                .output('pipe:', format='rawvideo', pix_fmt='rgb24')
                .overwrite_output()
                .run_async(pipe_stdout=True)
            )
            while True:
                self.in_bytes = self.process1.stdout.read(self.width * self.height * 3)     # 读取图片
                if not self.in_bytes:
                    break
                
                # 转成ndarray
                self.in_frame = (
                    np.frombuffer(self.in_bytes, np.uint8)
                    .reshape([self.height, self.width, 3])
                )
                # # in_frame = cv2.resize(in_frame, (1280, 720))   # 改变图片尺寸
                self.frame = cv2.cvtColor(self.in_frame, cv2.COLOR_BGR2RGB)  # 转成BGR
                self.bridge = CvBridge()
                self.publisher_.publish(self.bridge.cv2_to_imgmsg(self.frame))
                # self.publisher_.publish(self.bridge.cv2_to_imgmsg(self.frame))
                # cv2.imshow("ffmpeg", self.frame)
                # if cv2.waitKey(1) == ord('q'):
                #     break
            self.process1.kill()             # 关闭
        else:
            self.get_logger().warning("can't connect IPcam")
            while not self.probe:
                self.pub_img()



def main():
    rclpy.init()
    ipcamera_publisher = IPCmaeraPublisher() 
    rclpy.spin(ipcamera_publisher)
    ipcamera_publisher.destroy_node()
    rclpy.shutdown()



if __name__ == "__main__":
    main()
