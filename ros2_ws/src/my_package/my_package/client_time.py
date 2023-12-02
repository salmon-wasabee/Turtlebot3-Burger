from my_interfaces.srv import TimeCheck  # Fix the import statement
import sys
import rclpy
from rclpy.node import Node

class MinimalClientAsync(Node):
    def __init__(self):  # Fix the method name
        super().__init__('minimal_client_async')
        self.cli = self.create_client(TimeCheck, 'time_check')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')  # Fix the parentheses and quotes
        self.req = TimeCheck.Request()  # Fix the class name

    def send_request(self):
        # Gets the variables (arguments) typed at the command line
        self.req.digit = int(sys.argv[1])  # Check the Terminal 2, e.g., after building
        self.future = self.cli.call_async(self.req)

def main(args=None):
    rclpy.init(args=args)
    minimal_client = MinimalClientAsync()  # Fix the class instantiation
    minimal_client.send_request()
    
    while rclpy.ok():
        rclpy.spin_once(minimal_client)
        if minimal_client.future.done():
            try:
                response = minimal_client.future.result()
            except Exception as e:
                minimal_client.get_logger().info(
                'Service call failed %s' % (e,))  # Fix the parentheses and quotes
            else:
                minimal_client.get_logger().info(
                'Current Time: %s' % (response.time))  # Fix the parentheses and quotes
            break
    minimal_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

