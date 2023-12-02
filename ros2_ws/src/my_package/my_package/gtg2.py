import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from math import pow, atan2, sqrt

class GoToGoal(Node):

    def __init__(self, goal_x, goal_y):
        super().__init__('go_to_goal')
        self.publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.subscription = self.create_subscription(Pose, '/turtle1/pose', self.update_pose, 10)
        self.goal_pose = Pose()
        self.goal_pose.x = goal_x
        self.goal_pose.y = goal_y
        self.current_pose = Pose()
        self.goal_reached = False

    def update_pose(self, data):
        self.current_pose = data
        self.move_to_goal()

    def euclidean_distance(self):
        return sqrt(pow((self.goal_pose.x - self.current_pose.x), 2.0) + 
                    pow((self.goal_pose.y - self.current_pose.y), 2.0))

    def linear_vel(self, goal_pose, constant=1.5):
        return constant * self.euclidean_distance()

    def steering_angle(self):
        return atan2(self.goal_pose.y - self.current_pose.y, 
                     self.goal_pose.x - self.current_pose.x)

    def angular_vel(self, constant=6):
        return constant * (self.steering_angle() - self.current_pose.theta)

    def move_to_goal(self):
        vel_msg = Twist()
        distance_tolerance = 0.01
        if self.euclidean_distance() >= distance_tolerance:
            self.goal_reached = False
            vel_msg.linear.x = self.linear_vel(self.goal_pose)
            vel_msg.angular.z = self.angular_vel()
            self.publisher.publish(vel_msg)
        elif not self.goal_reached:
            self.goal_reached = True
            vel_msg.linear.x = 0.0
            vel_msg.angular.z = 0.0
            self.publisher.publish(vel_msg)
            print("Goal Reached")
            self.destroy_node()


def main(args=None):
    rclpy.init(args=args)
    while rclpy.ok():
        goal_x = float(input("Enter the x-coordinate of the goal: "))
        goal_y = float(input("Enter the y-coordinate of the goal: "))
        node = GoToGoal(goal_x, goal_y)
        rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()

