import rclpy as rp 
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge 
import cv2 

class ImgPublisher(Node):
    def __init__(self):
        super().__init__('img_publisher')
        self.publisher = self.create_publisher(Image, '/image_raw', 10)
        
        self.declare_parameter('camera_device', 0)
        self.camera_device = self.get_parameter('camera_device').value
        self.declare_parameter('width', 640)
        self.width = self.get_parameter('width').value
        self.declare_parameter('height', 480)
        self.height = self.get_parameter('height').value
        self.declare_parameter('frame_rate', 30)
        self.frame_rate = self.get_parameter('frame_rate').value
        
        self.cap = cv2.VideoCapture(0)
        self.cv_bridge = CvBridge()
        time_period = 1.0/self.frame_rate
        self.timer = self.create_timer(time_period, self.callback_timer)

        self.get_logger().info('Camera Device: '+str(self.camera_device))
        self.get_logger().info('Video Width: ' + str(self.width))
        self.get_logger().info('video Height: ' + str(self.height))
        self.get_logger().info('Frame rate: ' + str(self.frame_rate))

    def callback_timer(self):
        ret, frame = self.cap.read()                            # frame : openCV type
        frame = cv2.resize(frame, (self.width, self.height))    # this operation consumes CPU power
        img = self.cv_bridge.cv2_to_imgmsg(frame, 'bgr8')       # img : ROS type
        self.publisher.publish(img)

    def destory_node(self):
        self.cap.release()
        super().destory_node()

def main(args=None):
    rp.init(args=args)
    node = ImgPublisher()
    rp.spin(node)
    node.destory_node()
    rp.shutdown()

if __name__ == '__main__':
    main()
