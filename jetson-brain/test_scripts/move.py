# import rclpy
# from rclpy.node import Node
# from geometry_msgs.msg import Twist
# import time

# class JackalMover(Node):
#     def __init__(self):
#         super().__init__('jackal_mover')
#         self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
#         # time.sleep(1)  # Give ROS time to initialize

#         #Sends command evrey 0.5 seconds
#         self.timer = self.create_timer(0.5, self.move)

#     # def move_forward(self, speed=0.2, duration=3.0):
#     def move(self):
#         msg = Twist()
#         # msg.linear.x = speed
#         msg.linear.x = 0.2
#         msg.angular.z = 0.0   # No rotation (only move in linear plane)
#         self.publisher_.publish(msg)
#         self.get_logger().info('Moving Jackal forward!')
#         # time.sleep(duration)

#         # Stop after moving
#         # msg.linear.x = 0.0
#         # self.publisher_.publish(msg)

# def main():
#     rclpy.init()
#     node = JackalMover()
#     # mover = JackalMover()
#     rclpy.spin(node)
#     #Moves forward for 3 seconds
#     # mover.move_forward(speed=0.2, duration=3.0) 
#     # mover.destroy_node()
#     node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()


import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

def choose_jackal():
    print("Choose the Jackal to control:")
    print("1. Jackal1")
    print("2. Jackal2")
    choice = input("Enter your choice (1 or 2): ")
    if choice == '1':
        return '/jackal1/cmd_vel'
    elif choice == '2':
        return '/jackal2/cmd_vel'
    else:
        print("Invalid choice. Defaulting to Jackal1.")
        return '/jackal1/cmd_vel'

class JackalController(Node):
    def __init__(self, topic):
        super().__init__('jackal_controller')
        self.publisher = self.create_publisher(Twist, topic, 10)
        # self.timer = self.create_timer(1.0, self.publish_twist)
        self.timer = self.create_timer(0.1, self.publish_twist)

    def publish_twist(self):
        twist = Twist()
        twist.linear.x = 0.5  # Forward speed
        twist.angular.z = 0.5 # Rotational speed
        self.publisher.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    topic = choose_jackal()
    node = JackalController(topic)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
