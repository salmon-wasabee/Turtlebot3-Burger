import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
#from openCV import process_frame
import cv2

class ImagePublisher(Node):
    def __init__(self):
        super().__init__('image_publisher')
        self.publisher_ = self.create_publisher(Image, 'video_frames', 10)
        self.cap = self.get_camera()
        self.br = CvBridge()
        self.timer = self.create_timer(0.1, self.timer_callback)

    def get_camera(self):
        # Iterate over the range of device IDs
        for i in range(10):
            cap = cv2.VideoCapture(i)
            if cap.isOpened():
                print(f"Webcam connected to device ID {i}")
                return cap
        print("Could not open any webcams")
        exit()

    def timer_callback(self):
        ret, frame = self.cap.read()
#        processed_frame = process_frame(frame)  #remove openCV computation on pub
        if ret == True:
            self.publisher_.publish(self.br.cv2_to_imgmsg(frame))

def main(args=None):
    rclpy.init(args=args)
    image_publisher = ImagePublisher()
    rclpy.spin(image_publisher)
    image_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

