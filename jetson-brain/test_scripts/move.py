import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class JackalMover(Node):
    def __init__(self):
        super().__init__('jackal_mover')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        # time.sleep(1)  # Give ROS time to initialize

        #Sends command evrey 0.5 seconds
        self.timer = self.create_timer(0.5, self.move)

    # def move_forward(self, speed=0.2, duration=3.0):
    def move(self):
        msg = Twist()
        # msg.linear.x = speed
        msg.linear.x = 0.2
        msg.angular.z = 0.0   # No rotation (only move in linear plane)
        self.publisher_.publish(msg)
        self.get_logger().info('Moving Jackal forward!')
        # time.sleep(duration)

        # Stop after moving
        # msg.linear.x = 0.0
        # self.publisher_.publish(msg)

def main():
    rclpy.init()
    node = JackalMover()
    # mover = JackalMover()
    rclpy.spin(node)
    #Moves forward for 3 seconds
    # mover.move_forward(speed=0.2, duration=3.0) 
    # mover.destroy_node()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()