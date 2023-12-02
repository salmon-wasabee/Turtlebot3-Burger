import time

import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node

from action_tutorials_interfaces.action import Prime


class PrimeActionServer(Node):

    def __init__(self):
        super().__init__('prime_action_server')
        self._action_server = ActionServer(
            self,
            Prime,
            'prime',
            self.execute_callback)
        
    def isPrime(self, num):
    	i=2
    	while i < num:
    		if num%i == 0 :
    			return 0
    			exit()
    		i=i+1
    	return num      
    
    	
	
    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')
	
        feedback_msg = Prime.Feedback()
        feedback_msg.partial_list = []
        
        count = 1
        lengthPrime = 0
        
	# Script to compute prime numbers
        while (lengthPrime < goal_handle.request.number):
        	if self.isPrime(count) != 0:
        		feedback_msg.partial_list.append(self.isPrime(count))
        		lengthPrime += 1 
        	self.get_logger().info('Feedback: {0}'.format(feedback_msg.partial_list))
        	goal_handle.publish_feedback(feedback_msg)
        	time.sleep(1)
        	count = count + 1 

        goal_handle.succeed()

        result = Prime.Result()
        result.list = feedback_msg.partial_list
        return result



def main(args=None):
    rclpy.init(args=args)

    prime_action_server = PrimeActionServer()

    rclpy.spin(prime_action_server)


if __name__ == '__main__':
    main()
