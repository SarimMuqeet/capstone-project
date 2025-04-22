import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from std_msgs.msg import String
import json
import numpy as np

class ObjectTracker(Node):
    def __init__(self):
        super().__init__('object_tracker')
        # self.subscription = self.create_subscription(
        #     String,
        #     '/tracked_objects',
        #     self.tracked_objects_callback,
        #     10)
                
        self.subscription = self.create_subscription(
            String,
            "/tracked_objects",
            self.tracked_objects_callback,
            QoSProfile(
                reliability=QoSReliabilityPolicy.BEST_EFFORT,
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=10
            )
        )

        self.subscription  # prevent unused variable warning

    def tracked_objects_callback(self, msg):
        try:
            objects = json.loads(msg.data)
            self.get_logger().info(f"Received objects: {objects}")
            if objects:
                obj = objects[0]  # Take first detected object
                x, y, z = obj['x'], obj['y'], obj['z']
                self.get_logger().info(f"Raw coordinates: x = {x}, y = {y}, z = {z}")
                arm_coords = self.return_arm_coordinates(x, y, z)
                self.get_logger().info(f"Arm coordinates: {arm_coords[:3]}")
            else:
                self.get_logger().warn("Received empty object list")
        except Exception as e:
            self.get_logger().error(f"Object processing error: {str(e)}")

    def return_arm_coordinates(self, x, y, z):
        camera_input = np.array([x*100, y*100, z*100, 1])
        cam_to_global = np.array([
            [1, 0, 0, 0],
            [0, 1, 0, 0],
            [0, 0, 1, -7.5],
            [0, 0, 0, 1]
        ])
        global_to_arm = np.array([
            [1, 0, 0, -13.92],
            [0, 1, 0, 16],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ])
        global_coords = cam_to_global @ camera_input
        arm_coords = global_to_arm @ global_coords
        return arm_coords

def main(args=None):
    rclpy.init(args=args)
    object_tracker = ObjectTracker()
    rclpy.spin(object_tracker)
    object_tracker.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
