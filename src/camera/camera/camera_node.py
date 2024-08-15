'''
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
'''

import ffmpeg
import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class IPCameraPublisher(Node):
    def __init__(self):
        super().__init__('camera_node')

        # Declare and retrieve parameters
        self.declare_parameter('camera_ip', "192.168.1.108")
        self.declare_parameter('camera_login_user', "admin")
        self.declare_parameter('camera_login_pwd', "DAHUA123")
        self.declare_parameter('camera_channel', 0)

        self.camera_ip = self.get_parameter('camera_ip').get_parameter_value().string_value
        self.camera_login_user = self.get_parameter('camera_login_user').get_parameter_value().string_value
        self.camera_login_pwd = self.get_parameter('camera_login_pwd').get_parameter_value().string_value
        self.camera_channel = self.get_parameter("camera_channel").get_parameter_value().integer_value

        self.rtsp_url = (
            f"rtsp://{self.camera_login_user}:{self.camera_login_pwd}@{self.camera_ip}/"
            f"cam/realmonitor?channel=1&subtype={self.camera_channel}"
        )
        self.ffmpeg_args = {
            "rtsp_transport": "tcp",
            "fflags": "nobuffer",
            "flags": "low_delay"
        }

        self.publisher_ = self.create_publisher(Image, 'ipcamera', 10)
        self.bridge = CvBridge()
        self.publish_image()

    def publish_image(self):
        self.get_logger().info("Connecting to IP camera at " + self.rtsp_url)
        
        # Try to probe the camera stream
        try:
            probe = ffmpeg.probe(self.rtsp_url)
        except ffmpeg.Error as e:
            self.get_logger().warning("FFmpeg error: " + str(e))
            probe = None

        if not probe:
            self.get_logger().warning("Unable to connect to IP camera. Retrying...")
            self.get_clock().sleep_for(rclpy.duration.Duration(seconds=1))
            return self.publish_image()  # Retry connection

        # Retrieve stream information
        self.get_logger().info("Connected to IP camera: " + probe['format']["filename"])
        video_stream = next(stream for stream in probe['streams'] if stream['codec_type'] == 'video')
        width, height = video_stream['width'], video_stream['height']

        process = (
            ffmpeg
            .input(self.rtsp_url, **self.ffmpeg_args)
            .output('pipe:', format='rawvideo', pix_fmt='rgb24')
            .run_async(pipe_stdout=True)
        )

        try:
            while rclpy.ok():  # Loop runs while ROS is running
                in_bytes = process.stdout.read(width * height * 3)
                if not in_bytes:
                    self.get_logger().warning("No more frames received from camera.")
                    break  # Exit the loop if no frames are received

                frame = np.frombuffer(in_bytes, np.uint8).reshape([height, width, 3])
                # No need to convert BGR to RGB if it's already in RGB format

                try:
                    image_msg = self.bridge.cv2_to_imgmsg(frame, encoding="rgb8")
                    self.publisher_.publish(image_msg)
                except CvBridgeError as e:
                    self.get_logger().error("CV Bridge Error: " + str(e))

        finally:
            process.kill()  # Ensure process is terminated when the loop ends

def main():
    rclpy.init()
    camera_publisher = IPCameraPublisher()
    try:
        rclpy.spin(camera_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        camera_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
