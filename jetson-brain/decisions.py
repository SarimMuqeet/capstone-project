

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
        self.reachThreshold=0.2 # NEW, prev 0.2 for lab

        # TODO PART 5 your localization type
        #self.localizer=localization(type=kalmanFilter)
        self.localizer=localization(type=rawSensors)


        #NEW default is trajector planner (use A* based on cost_map)
        # if motion_type==POINT_PLANNER:
        #     self.controller=controller(klp=0.2, klv=0.5, kap=0.8, kav=0.6)      
        #     self.planner=planner(POINT_PLANNER)

        # elif motion_type==TRAJECTORY_PLANNER:
        #     # TODO PART 5 Bonus Put the gains that you conclude from lab 2
        #     self.controller=trajectoryController(klp = 0.4, klv = 0.5, kli=0.2, kap = 0.4, kav=0.2, kai=0.2)      
        #     self.planner=planner(TRAJECTORY_PLANNER)
        
        # else:
        #     print("Error! you don't have this type of planner", file=sys.stderr)


        # NEW - init trajectory planner by default
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


        #initialize timeout approx for pick and place tasks
        self.pick_start_time = None
        self.pick_duration = 3.0
        self.place_start_time = None
        self.place_duration = 3.0



        self.goal = None

        self.create_timer(publishing_period, self.timerCallback)


        # print("waiting for your input position, use 2D nav goal in rviz2")


    # NEW 
    def run_object_detection(self):
        #simulation object detection for now
        print("Simulating object detection...")
        predefined_objects = [
            # (x, y, z) coordinates
            #out of bounds?
            # (2.0, 3.0, 0.0), 
            # (-1.5, 4.5, 0.0), 
            # (1.0, -2.5, 0.0) 

            # NEW
            # (0.2, 0.2, 0.03), 

            (2, 2, 0.03),
            # CURR TEST
            (2.0, 1.0, 0.0)

            # (0.2, 0.2, 0.03), 
            # (0.2, -0.7, 0.03), 
            # (0.5, 0.5, 0.0), 
            # (1.0, -2.5, 0.0) 
        ]


        # # Debugging Filter out of bounds
        # valid_objects = []
        # max_rows, max_cols = self.planner.costMap.shape
        # for obj in predefined_objects:
        #     cell = self.planner.m_utilites.position_2_cell(obj[:2])
        #     if 0 <= cell[0] < max_rows and 0 <= cell[1] < max_cols:
        #         valid_objects.append(obj)
        #     else:
        #         print(f"Object {obj} is out of bounds (cell: {cell})")

        # return valid_objects

        return predefined_objects
    
    # NEW 
    def pick_up_object(self):
        #send pick command to STM32
        print("Sending pickup command to STM32...")
        if not self.object_queue:
            self.get_logger().warn("No objects to pick")
            return False
        
        #obtain object location from queue
        x, y, z = self.object_queue[0]

        #send PICK command (type 0)
        self.uart.send_command(0, x, y, z)

        #manual delay to allow for operation?
        # time.sleep(2)

        return True

    # NEW
    def place_object(self):
        #send place command to STM32
        print("Sending placement command to STM32...")
        if not self.drop_off_locations:
            self.get_logger().warn("No drop-off locations configured")
            return False

        x, y, z = self.drop_off_locations[self.current_drop_off_index ]
        
        #send PLACE command (type 1)
        self.uart.send_command(1, x, y, z)


        return True



    # This is for the rviz2 interface
    def designPathFor(self, msg: PoseStamped):
        
        spin_once(self.localizer)
        
        if self.localizer.getPose() is  None:
            print("waiting for odom msgs ....")
            return
        
        self.goal=self.planner.plan([self.localizer.getPose()[0], self.localizer.getPose()[1]],
                                     [msg.pose.position.x, msg.pose.position.y])

    
    def timerCallback(self):
        #implement state machine handling here

        print("inside timer callback start")

        #TO CHANGE / UNCOMMENT LATER: commented out for now as no odom simulation
        #  ------------------------------------ 
        # 
        # To use on actual robot:      
        spin_once(self.localizer)

        if self.localizer.getPose() is None:
            print("waiting for odom msgs ....")
            return
        




        #------------------------- OLD --------------------------
        # vel_msg=Twist()
        
        # if self.goal is None:
        #     return
        
        # if type(self.goal) == list:
        #     reached_goal=True if calculate_linear_error(self.localizer.getPose(), self.goal[-1]) <self.reachThreshold else False
        # else: 
        #     reached_goal=True if calculate_linear_error(self.localizer.getPose(), self.goal) <self.reachThreshold else False




        # if reached_goal:
        #     print("reached goal")
        #     self.publisher.publish(vel_msg)
            
        #     self.controller.PID_angular.logger.save_log()
        #     self.controller.PID_linear.logger.save_log()


            
        #     self.goal = None
        #     print("waiting for the new position input, use 2D nav goal on map")

        #     return
        
        # velocity, yaw_rate = self.controller.\
        #     vel_request(self.localizer.getPose(), self.goal, True)

        
        # vel_msg.linear.x=velocity
        # vel_msg.angular.z=yaw_rate
        
        # self.publisher.publish(vel_msg)
        # self.publishPathOnRviz2(self.goal)

        #-------------------------------------------------------------------------



        # NEW ----------------------------------------------------------------
        if not hasattr(self, 'state_machine'):
            print("State machine not initialized")
            return
        print("in timer callback")

        # ----------------- STATE MACHINE LOGIC -----------------
        current_pose = self.localizer.getPose()

        # Handle states using the state machine
        if self.state_machine.state == "IDLE":
            print("State: IDLE")
            # Initialize LiDAR, camera sensors and STM32 communication here


            # Go into IDENTIFY state immediately for now (may have random roam around first)
            self.state_machine.detect_objects()

        elif self.state_machine.state == "IDENTIFY":
            print("State: IDENTIFY")
            # Run object detection and add objects to queue
            detected_objects = self.run_object_detection()
            if detected_objects:
                for obj in detected_objects:
                    #add detected objects to queue
                    self.object_queue.append(obj)
                self.state_machine.plan_path_to_object()

        #once object detected, path plan to get to it
        elif self.state_machine.state == "TO_OBJECT":
            print("State: TO_OBJECT")
            if not self.object_queue:
                print("No objects in queue.")
                return

            # Plan path to the next object in the queue
            target_object = self.object_queue[0]
            print(f"target object (x,y): {target_object[:2]}")
            print(f"current pose: {current_pose[:2]}")
            path = self.planner.plan(current_pose[:2], target_object[:2])
            print(f"path planned from: {current_pose[:2]} to {target_object[:2]}")

            if path:
                print(f"Path planned to object, path is not empty!")
                # Publish path for visualization in RViz
                self.publishPathOnRviz2(path)

                velocity, yaw_rate = self.controller.vel_request(current_pose, path, True)
                vel_msg = Twist()
                vel_msg.linear.x = velocity
                vel_msg.angular.z = yaw_rate
                self.publisher.publish(vel_msg)

                # # Check if goal is reached
                if calculate_linear_error(current_pose, target_object) < self.reachThreshold:
                    print("Reached object.")
                    self.publisher.publish(Twist())  # Stop robot
                    self.state_machine.pick_object()

                #simulate just moving to next state (in simulation)
                # self.state_machine.pick_object()


        elif self.state_machine.state == "PICK":
            # print("State: PICK\n")
            # # Send commands to STM32 to pick up the object
            # success = self.pick_up_object()

            # if success:
            #     print("Object picked up.\n")
            #     # Remove object from queue to prevent double servicing
            #     del self.object_queue[0]
            #     # Find tidy destination - hardcoded for now
            #     x_dropoff, y_dropoff, z_dropoff = self.drop_off_locations[self.current_drop_off_index]
            #     print(f"Assigned drop-off location: ({x_dropoff}, {y_dropoff}, {z_dropoff})")
            #     self.goal = (x_dropoff, y_dropoff)

            #     # working but too fast ---
            #     self.state_machine.plan_path_to_destination()

            #NEW - with configured timer for task
            if self.pick_start_time is None:  # First entry to PICK state
                print("BEGINNING PICK OPERATION")
                self.pick_up_object()
                self.pick_start_time = self.get_clock().now()  # ROS2 time object
            else:
                # Calculate elapsed time in seconds
                elapsed = (self.get_clock().now() - self.pick_start_time).nanoseconds * 1e-9
                
                print(f"Picking... {self.pick_duration - elapsed:.1f}s remaining")
                
                if elapsed >= self.pick_duration:
                    print("PICK OPERATION COMPLETE")
                    del self.object_queue[0]
                    x_dropoff, y_dropoff, z_dropoff = self.drop_off_locations[self.current_drop_off_index]
                    print(f"Assigned drop-off location: ({x_dropoff}, {y_dropoff}, {z_dropoff})")
                    self.goal = (x_dropoff, y_dropoff)

                    self.pick_start_time = None
                    self.state_machine.plan_path_to_destination()


        elif self.state_machine.state == "TO_TIDY_DESTINATION":
            print("State: TO_TIDY_DESTINATION")
            
            path_to_destination = self.planner.plan(current_pose[:2], self.goal)
            
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
                    print("Reached tidy destination.")
                    self.publisher.publish(Twist())  # Stop robot
                    self.state_machine.place_object()

                #simulate just moving to next state (in simulation)
                # self.state_machine.place_object()

        elif self.state_machine.state == "PLACE_OBJECT":
            # print("State: PLACE_OBJECT")
            
            # success = self.place_object()
            
            # if success:
            #     print("Object placed successfully.")
            #     #update drop_off_index to next in list:
            #     self.current_drop_off_index = (self.current_drop_off_index + 1) % len(self.drop_off_locations)
                
            #     #if all objects serviced (queue empty)
            #     if not len(self.object_queue): 
            #         self.state_machine.review_objects()  # Transition to REVIEW
            #     else:
            #         #if array not empty, plan path to the next object in the queue
            #         self.state_machine.plan_next_object()

            #NEW - with configured timer for task
            if self.place_start_time is None:  # First entry to PLACE state
                print("BEGINNING PLACE OPERATION")
                self.place_object()
                #update drop_off_index to next in list:
                self.current_drop_off_index = (self.current_drop_off_index + 1) % len(self.drop_off_locations)
                self.place_start_time = self.get_clock().now()  # ROS2 time object
            else:
                # Calculate elapsed time in seconds
                elapsed = (self.get_clock().now() - self.place_start_time).nanoseconds * 1e-9
                
                print(f"Placing... {self.place_duration - elapsed:.1f}s remaining")
                
                if elapsed >= self.place_duration:
                    print("PLACE OPERATION COMPLETE")

                    self.place_start_time = None
                    #if all objects serviced (queue empty)
                    if not len(self.object_queue): 
                        self.state_machine.review_objects()  # Transition to REVIEW
                    else:
                        #if array not empty, plan path to the next object in the queue
                        self.state_machine.plan_next_object()


        elif self.state_machine.state == "REVIEW":
            print("State: REVIEW")

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
