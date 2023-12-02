import rospy
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Twist

# Initialize ROS node
rospy.init_node('move_around_person')

# Create a publisher for sending velocity commands
pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

# Create a SimpleActionClient for MoveBaseAction
client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
client.wait_for_server()

# Process the frame and get the bounding box of the person
# This part is already done in your current code

# Map the person's position to the robot's world
# This would be specific to your application

# Plan the path around the person using A* algorithm
# This would involve creating a MoveBaseGoal with the target position
goal = MoveBaseGoal()
goal.target_pose.header.frame_id = "base_link"
goal.target_pose.header.stamp = rospy.Time.now()

# Set the target position
goal.target_pose.pose.position.x = target_x
goal.target_pose.pose.position.y = target_y
goal.target_pose.pose.orientation.w = 1.0

# Send the goal to MoveBaseAction server
client.send_goal(goal)

# Wait for the result
client.wait_for_result()

# If the robot reaches the target, stop the robot
if client.get_state() == GoalStatus.SUCCEEDED:
    vel_msg = Twist()
    pub.publish(vel_msg)
