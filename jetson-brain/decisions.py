

import sys


from utilities import euler_from_quaternion, calculate_angular_error, calculate_linear_error
from pid import PID_ctrl

from rclpy import init, spin, spin_once
from rclpy.node import Node
from geometry_msgs.msg import Twist


from rclpy.qos import QoSProfile
from nav_msgs.msg import Odometry as odom

from localization import localization, rawSensors, kalmanFilter

from planner import TRAJECTORY_PLANNER, POINT_PLANNER, planner
from controller import controller, trajectoryController


from geometry_msgs.msg import PoseStamped


from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

#to use state machine
from state_machine import RobotStateMachine

#for UART command message construction
# import struct
from uart_transmitter import UART_Transmitter

#for obj detection parsing
import json
from std_msgs.msg import String 
import numpy as np
import threading

from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

#for timeout for PICK, PLACE commands
import time

class decision_maker(Node):
    
    
    def __init__(self, publisher_msg, publishing_topic, qos_publisher, rate=10, motion_type=POINT_PLANNER):

        super().__init__("decision_maker")

        self.publisher=self.create_publisher(publisher_msg, publishing_topic, qos_profile=qos_publisher)


        self.create_subscription(PoseStamped, "/goal_pose", self.designPathFor, 10)
        
        self.pathPublisher = self.create_publisher(Path, '/designedPath', 10)
        
        publishing_period=1/rate

        # TODO PART 5 choose your threshold
        # NEW increased threshold to account for arm length (when pick and place occurring)
        self.reachThreshold=0.3 # NEW, prev 0.2 for lab

        # TODO PART 5 your localization type
        #self.localizer=localization(type=kalmanFilter)
        self.localizer=localization(type=rawSensors)

        #tune for PID
        # self.controller=trajectoryController(klp = 0.3, klv = 0.3, kli=0.3, kap = 0.4, kav=0.2, kai=0.2) 
        self.controller=trajectoryController(klp = 0.4, klv = 0.5, kli=0.2, kap = 0.4, kav=0.2, kai=0.2)      
        self.planner=planner(TRAJECTORY_PLANNER)
        # self.controller=controller(klp=0.2, klv=0.5, kap=0.8, kav=0.6)  
        # self.planner=planner(POINT_PLANNER)

        # NEW create queue for detected objects with positions
        self.object_queue = []
        # Hardcoded Drop-off Locations for Now
        # self.drop_off_locations = [[5.0, 5.0], [6.0, 6.0], [7.0, 7.0]]
        self.drop_off_locations = [
            #(x, y, z) coordinates for dropoff
            #out of bounds?
            # (5.0, 5.0, 0.5), 
            # (5.1, 5.3, 0.3), 
            # (7.0, 7.0, 0.7)

            # NEW:
            # (1, 1, 0.5), #1st working, try another:
            # (2, 0, 0.5), #works,
            # (3.5, 2, 0.5), #works
            (4.5, 2, 0.5),
            # (1, 2, 0.5),
            #CURR TEST
            (0, 0, 0.7)
            # (0.1, 0.3, 0.3), 
            # (0.4, 0.0, 0.7)
        ]
        
        #to keep track of index for drop off locations
        self.current_drop_off_index = 0

        #create instance of robot state machine to use
        self.state_machine = RobotStateMachine()

        # NEW - initialize UART Transmitter object for sending msgs
        self.uart = UART_Transmitter(port="/dev/ttyTHS1", baudrate=115200)

        #LISTEN on OBJECT DETECTION NODE:
                # Add subscriber for tracked objects
        # self.create_subscription(String, "/tracked_objects", self.tracked_objects_callback, 10)
        
        self.create_subscription(
            String,
            "/tracked_objects",
            self.tracked_objects_callback,
            QoSProfile(
                reliability=QoSReliabilityPolicy.BEST_EFFORT,
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=10
            )
        )

        self.latest_object = None  # Stores 3x1 vector of latest detection
        self.object_lock = threading.Lock()  # Thread-safe access

        # Initialize pick_attempt_count in __init__
        self.pick_attempt_count = 0

        #initialize timeout approx for pick and place tasks
        self.pick_start_time = None
        self.pick_duration = 10.0
        self.place_start_time = None
        self.place_duration = 25.0



        self.goal = None

        self.create_timer(publishing_period, self.timerCallback)
        
    def tracked_objects_callback(self, msg):
        try:
            objects = json.loads(msg.data)
            self.get_logger().info(f"Received objects: {objects}")
            if objects:
                obj = objects[0]  # Take first detected object
                with self.object_lock:
                    x = obj['x']
                    y = obj['y']
                    z = obj['z']
                    self.get_logger().info(f"Raw coordinates: x = {x}, y = {y}, z = {z}")
                    arm_coords = self.return_arm_coordinates(x, y, z)
                    self.latest_object = arm_coords[:3]
                    self.get_logger().info(f"Updated latest object: {self.latest_object}")
            else:
                self.get_logger().warn("Received empty object list")
        except Exception as e:
            self.get_logger().error(f"Object processing error: {str(e)}")






    def return_arm_coordinates(self, x, y, z):
        """Convert camera coordinates to arm coordinates"""
        # Create homogeneous input vector
        camera_input = np.array([x*100, y*100, z*100, 1])
        
        # Camera to global transformation 
        theta = -70
        cam_to_global = np.array([
            [1, 0, 0, 0],
            [0, 1, 0, 0],
            [0, 0, 1, -10],
            [0, 0, 0, 1]
        ])
        
        # Global to arm transformation
        global_to_arm = np.array([
            [1, 0, 0, -12.5],
            [0, 1, 0, 16],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ])
        
        # Apply transformations
        global_coords = cam_to_global @ camera_input
        arm_coords = global_to_arm @ global_coords
        
        return arm_coords

    # def pick_up_object(self):
    #     if self.pick_attempt_count >= 10:
    #         self.get_logger().warn("No object available for picking after 20 attempts")
    #         return False

    #     with self.object_lock:
    #         if self.latest_object is not None:
    #             x, y, z = self.latest_object
    #             self.get_logger().info(f"Sending pick command for coordinates: {x:.2f}, {y:.2f}, {z:.2f}")
    #             self.uart.send_command(0, x, z, y)
    #             self.latest_object = None
    #             return True

    #     self.get_logger().info("No object available, will retry in 1 second")
    #     self.pick_attempt_count += 1
    #     self.create_timer(1.0, self.pick_up_object)
    #     return False

    # def pick_up_object(self):
    #     with self.object_lock:
    #         if self.latest_object is not None:
    #             x, y, z = self.latest_object
    #             self.get_logger().info(f"Object coordinates: x = {x:.2f}, y = {y:.2f}, z = {z:.2f}")
                
    #             if z > 15:
    #                 self.get_logger().info("Object is far enough. Sending pick command.")
    #                 self.uart.send_command(0, x, z, y)
    #                 self.latest_object = None
    #                 self.pick_attempt_count = 0
    #                 return True
    #             else:
    #                 self.get_logger().info("Object is too close. Not sending pick command.")
    #                 return False

    #     self.get_logger().info("No object available")
    #     return False


    def pick_up_object(self):
        with self.object_lock:
            if self.latest_object is not None:
                x, y, z = self.latest_object
                self.get_logger().info(f"Sending pick command for coordinates: {x:.2f}, {y:.2f}, {z:.2f}")
                self.uart.send_command(0, x, z, y)
                return True
                # self.latest_object = None
                # return True
        self.get_logger().warn("No object available for picking")
        return False








    # NEW 
    def run_object_detection(self):
        #simulation object detection for now
        # print("Simulating object detection...")
        predefined_objects = [
                # (x, y, z) coordinates
                #out of bounds?
                # (2.0, 3.0, 0.0), 
                # (-1.5, 4.5, 0.0), 
                # (1.0, -2.5, 0.0) 


            (2, 2, 0.03),
            # CURR TEST
            (2.0, 1.0, 0.0)

        ]

        return predefined_objects
      
    
    # NEW 
    # def pick_up_object(self):
    #     # #send pick command to STM32
    #     # print("Sending pickup command to STM32...")
    #     # if not self.object_queue:
    #     #     self.get_logger().warn("No objects to pick")
    #     #     return False
        
    #     # #obtain object location from queue
    #     # x, y, z = self.object_queue[0
    #     # #read xyz from the object detection node array being returned...

    #     # #send PICK command (type 0)
    #     # self.uart.send_command(0, x, y, z)


    #     """Retrieve object and send UART command"""
    #     with self.object_lock:
    #         if self.latest_object is None:
    #             self.get_logger().warn("No object available for picking")
    #             return False
            
    #         # Convert 4x1 homogeneous to 3D coordinates
    #         transformed_coords = self.latest_object[:3].flatten()
    #         x, y, z = transformed_coords[0], transformed_coords[1], transformed_coords[2]
            
    #         # Send UART command
    #         self.uart.send_command(0, x, y, z)
            
    #         # Clear object after use
    #         self.latest_object = None
    #         return True

    #     return True

    # NEW
    def place_object(self):
        #send place command to STM32
        print("Sending placement command to STM32...")
        if not self.drop_off_locations:
            self.get_logger().warn("No drop-off locations configured")
            return False

        x, y, z = self.drop_off_locations[self.current_drop_off_index ]
        
        #hardcode dropoff

        #send PLACE command (type 1)
        self.uart.send_command(1, x, y, z)


        return True



    # This is for the rviz2 interface
    def designPathFor(self, msg: PoseStamped):
        
        spin_once(self.localizer)
        
        if self.localizer.getPose() is  None:
            # print("waiting for odom msgs ....")
            return
        
        self.goal=self.planner.plan([self.localizer.getPose()[0], self.localizer.getPose()[1]],
                                     [msg.pose.position.x, msg.pose.position.y])

    
    def timerCallback(self):
        #implement state machine handling here

        # print("inside timer callback start")

        #TO CHANGE / UNCOMMENT LATER: commented out for now as no odom simulation
        #  ------------------------------------ 
        # 
        # To use on actual robot:      
        spin_once(self.localizer)

        if self.localizer.getPose() is None:
            # print("waiting for odom msgs ....")
            return

        # NEW ----------------------------------------------------------------
        if not hasattr(self, 'state_machine'):
            # print("State machine not initialized")
            return
        # print("in timer callback")

        # ----------------- STATE MACHINE LOGIC -----------------
        current_pose = self.localizer.getPose()

        # Handle states using the state machine
        if self.state_machine.state == "IDLE":
            # print("State: IDLE")
            # Initialize LiDAR, camera sensors and STM32 communication here


            # Go into IDENTIFY state immediately for now (may have random roam around first)
            self.state_machine.detect_objects()

        elif self.state_machine.state == "IDENTIFY":
            # print("State: IDENTIFY")
            # Run object detection and add objects to queue
            detected_objects = self.run_object_detection()
            if detected_objects:
                for obj in detected_objects:
                    #add detected objects to queue
                    self.object_queue.append(obj)
                self.state_machine.plan_path_to_object()

                #to test pick in just on eplace
                # self.state_machine.test_pick()

        #once object detected, path plan to get to it
        elif self.state_machine.state == "TO_OBJECT":
            # print("State: TO_OBJECT")
            if not self.object_queue:
                print("No objects in queue.")
                return

            # Plan path to the next object in the queue
            target_object = self.object_queue[0]
            # print(f"target object (x,y): {target_object[:2]}")
            # print(f"current pose: {current_pose[:2]}")
            path = self.planner.plan(current_pose[:2], target_object[:2])
            # print(f"path planned from: {current_pose[:2]} to {target_object[:2]}")

            if path:
                # print(f"Path planned to object, path is not empty!")
                # Publish path for visualization in RViz
                self.publishPathOnRviz2(path)

                velocity, yaw_rate = self.controller.vel_request(current_pose, path, True)
                vel_msg = Twist()
                vel_msg.linear.x = velocity
                vel_msg.angular.z = yaw_rate
                self.publisher.publish(vel_msg)

                # # Check if goal is reached
                if calculate_linear_error(current_pose, target_object) < self.reachThreshold:
                    # print("Reached object.")
                    self.publisher.publish(Twist())  # Stop robot
                    self.state_machine.pick_object()

                #simulate just moving to next state (in simulation)
                # self.state_machine.pick_object()


        elif self.state_machine.state == "PICK":
            if self.pick_start_time is None:  # First entry to PICK state
                self.get_logger().info("BEGINNING PICK OPERATION")
                success = self.pick_up_object()
                if success:
                    self.pick_start_time = self.get_clock().now()
                    self.get_logger().info("Pick command sent successfully")
                else:
                    self.get_logger().info("Pick command not sent, object too close or not available")
                    # Optionally, you can add a small delay here before the next attempt
                    # self.create_timer(1.0, self.timerCallback)
                    return
            else:
                elapsed = (self.get_clock().now() - self.pick_start_time).nanoseconds * 1e-9
                if elapsed >= self.pick_duration:
                    self.get_logger().info("PICK OPERATION COMPLETE")
                    if self.object_queue:
                        del self.object_queue[0]
                    x_dropoff, y_dropoff, z_dropoff = self.drop_off_locations[self.current_drop_off_index]
                    self.goal = (x_dropoff, y_dropoff)
                    self.pick_start_time = None
                    self.state_machine.plan_path_to_destination()

        # elif self.state_machine.state == "PICK":
        #     if self.pick_start_time is None:  # First entry to PICK state
        #         self.get_logger().info("BEGINNING PICK OPERATION")
        #         success = self.pick_up_object()
        #         self.pick_start_time = self.get_clock().now()
        #         if success:
        #             self.get_logger().info("Pick command sent successfully")
        #         else:
        #             self.get_logger().warn("Failed to send pick command")
        #     else:
        #         elapsed = (self.get_clock().now() - self.pick_start_time).nanoseconds * 1e-9
        #         if elapsed >= self.pick_duration:
        #             self.get_logger().info("PICK OPERATION COMPLETE")
        #             del self.object_queue[0]
        #             x_dropoff, y_dropoff, z_dropoff = self.drop_off_locations[self.current_drop_off_index]
        #             self.goal = (x_dropoff, y_dropoff)
        #             self.pick_start_time = None
        #             self.pick_attempt_count = 0  # Reset the attempt count
        #             self.state_machine.plan_path_to_destination()



        elif self.state_machine.state == "TO_TIDY_DESTINATION":
            # print("State: TO_TIDY_DESTINATION")
            
            path_to_destination = self.planner.plan(current_pose[:2], self.goal)
            # print(f"tidy destination (x,y): {self.goal}")
            # print(f"current pose: {current_pose[:2]}")
            # print(f"path planned from: {current_pose[:2]} to {self.goal}")


            if path_to_destination:
                #publish to rviz
                self.publishPathOnRviz2(path_to_destination)

                velocity, yaw_rate = self.controller.vel_request(current_pose, path_to_destination, True)
                vel_msg = Twist()
                vel_msg.linear.x = velocity
                vel_msg.angular.z = yaw_rate
                self.publisher.publish(vel_msg)

                #Check if goal is reached
                if calculate_linear_error(current_pose, self.goal) < self.reachThreshold:
                    # print("Reached tidy destination.")
                    self.publisher.publish(Twist())  # Stop robot
                    self.state_machine.place_object()

                #simulate just moving to next state (in simulation)
                # self.state_machine.place_object()

        elif self.state_machine.state == "PLACE_OBJECT":


            #NEW - with configured timer for task
            if self.place_start_time is None:  # First entry to PLACE state
                # print("BEGINNING PLACE OPERATION")
                self.place_object()
                #update drop_off_index to next in list:
                self.current_drop_off_index = (self.current_drop_off_index + 1) % len(self.drop_off_locations)
                self.place_start_time = self.get_clock().now()  # ROS2 time object
            else:
                # Calculate elapsed time in seconds
                elapsed = (self.get_clock().now() - self.place_start_time).nanoseconds * 1e-9
                
                # print(f"Placing... {self.place_duration - elapsed:.1f}s remaining")
                
                if elapsed >= self.place_duration:
                    # print("PLACE OPERATION COMPLETE")

                    self.place_start_time = None
                    #if all objects serviced (queue empty)
                    if not len(self.object_queue): 
                        self.state_machine.review_objects()  # Transition to REVIEW
                    else:
                        #if array not empty, plan path to the next object in the queue
                        self.state_machine.plan_next_object()


        elif self.state_machine.state == "REVIEW":
            # print("State: REVIEW")

            print("All objects tidied. Task complete.")
            
            # if not self.object_queue:
            #     print("All objects tidied. Task complete.")
            #     # Stop the robot
            #     self.publisher.publish(Twist())
            #     # Optional: Shutdown the node or exit
            #     # rclpy.shutdown()
            # else:
            #     # If objects are added later, transition back to IDENTIFY
            #     self.state_machine.detect_objects()



    def publishPathOnRviz2(self, path):

        Path_ =  Path()

        Path_.header.frame_id ="map"
        Path_.header.stamp = self.get_clock().now().to_msg()

        for point in path:
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position.x = point[0]
            pose.pose.position.y = point[1]

            # Set the orientation of the pose. Here, it's set to a default orientation.
            pose.pose.orientation.x = 0.0
            pose.pose.orientation.y = 0.0
            pose.pose.orientation.z = 0.0
            pose.pose.orientation.w = 1.0

            Path_.poses.append(pose)

        self.pathPublisher.publish(Path_)

import argparse
def main(args=None):
    
    
    init()
    
    odom_qos=QoSProfile(reliability=2, durability=2, history=1, depth=10)
    
    # For Simulation

    # if args.motion == "point":
    #     DM=decision_maker(Twist, "/cmd_vel", 10, motion_type=POINT_PLANNER)
    # elif args.motion == "trajectory":
    #     DM=decision_maker(Twist, "/cmd_vel", 10, motion_type=TRAJECTORY_PLANNER)
    # else:
    #     print("invalid motion type", file=sys.stderr)


    # NEW for jackal1 cmd_vel topic
    DM=decision_maker(Twist, "/jackal1/cmd_vel", 10, motion_type=TRAJECTORY_PLANNER)
    # DM=decision_maker(Twist, "/jackal1/cmd_vel", 10, motion_type=POINT_PLANNER)



    try:
        print("main func - spin DM")
        spin(DM)
    except SystemExit:
        print(f"reached there successfully {DM.localizer.pose}")




if __name__=="__main__":
    argParser=argparse.ArgumentParser(description="point or trajectory") 
    argParser.add_argument("--motion", type=str, default="trajectory")
    args = argParser.parse_args()

    main(args)
