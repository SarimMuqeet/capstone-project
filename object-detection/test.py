import rclpy
import time
from rclpy.node import Node
from std_msgs.msg import String

def main():
    rclpy.init()
    node = Node('test_node')
    pub = node.create_publisher(String, 'test_topic', 10)
    
    print("Publishing to test_topic...")
    while rclpy.ok():
        msg = String()
        msg.data = "test_" + str(time.time())
        pub.publish(msg)
        print(f"Published: {msg.data}")
        rclpy.spin_once(node, timeout_sec=0.1)  # 👈 CRITICAL ADDITION
        time.sleep(1)

if __name__ == '__main__':
    main()