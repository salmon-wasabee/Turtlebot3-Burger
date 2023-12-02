from my_interface.srv import TimeCheck

import time
import datetime
import rclpy

from rclpy.node import Node



class MinimalService(Node):

    def __init__(self):
        super().__init__('minimal_service')
        self.srv = self.create_service(TimeCheck, 'time_check',
        self.time_check_callback)

    def time_check_callback(self, request, response):
    	response.time = datetime.datetime.now().strftime("%H:%M:%S")
        self.get_logger().info('Incoming request\nTimecheck (bool): %d' % (request.digit))
        return response


def main(args=None):
    rclpy.init(args=args)
    minimal_service = MinimalService()
    rclpy.spin(minimal_service)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
