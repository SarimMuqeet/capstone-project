

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

class decision_maker(Node):
    
    
    def __init__(self, publisher_msg, publishing_topic, qos_publisher, rate=10, motion_type=POINT_PLANNER):

        super().__init__("decision_maker")

        self.publisher=self.create_publisher(publisher_msg, publishing_topic, qos_profile=qos_publisher)


        self.create_subscription(PoseStamped, "/goal_pose", self.designPathFor, 10)
        
        self.pathPublisher = self.create_publisher(Path, '/designedPath', 10)
        
        publishing_period=1/rate

        # TODO PART 5 choose your threshold
        # NEW increased threshold to account for arm length (when pick and place occurring)
        self.reachThreshold=0.2

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

        self.controller=trajectoryController(klp = 0.4, klv = 0.5, kli=0.2, kap = 0.4, kav=0.2, kai=0.2)      
        self.planner=planner(TRAJECTORY_PLANNER)

        # NEW create queue for detected objects with positions
        self.object_queue = []
        # Hardcoded Drop-off Locations for Now
        # self.drop_off_locations = [[5.0, 5.0], [6.0, 6.0], [7.0, 7.0]]
        self.drop_off_locations = [
            #(x, y, z) coordinates for dropoff
            (5.0, 5.0, 0.5), 
            (6.0, 6.0, 0.3), 
            (7.0, 7.0, 0.7)
        ]
        
        #to keep track of index for drop off locations
        self.current_drop_off_index = 0



        self.goal = None

        self.create_timer(publishing_period, self.timerCallback)


        print("waiting for your input position, use 2D nav goal in rviz2")


    # NEW 
    def run_object_detection(self):
        #sim object detection for now
        print("Simulating object detection...")
        predefined_objects = [
            # (x, y) coordinates
            (2.0, 3.0), 
            (-1.5, 4.5), 
            (1.0, -2.5) 
        ]
        return predefined_objects
    
    # NEW 
    def pick_up_object(self):
        #send pick command to STM32
        print("Sending pickup command to STM32...")


        return True

    # NEW
    def place_object(self):
        #send place command to STM32
        print("Sending placement command to STM32...")


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
        
        spin_once(self.localizer)

        if self.localizer.getPose() is  None:
            print("waiting for odom msgs ....")
            return
        
        
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




        # NEW 
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
            path = self.planner.plan(current_pose[:2], target_object[:2])
            
            if path:
                # Publish path for visualization in RViz
                self.publishPathOnRviz2(path)

                velocity, yaw_rate = self.controller.vel_request(current_pose, path)
                vel_msg = Twist()
                vel_msg.linear.x = velocity
                vel_msg.angular.z = yaw_rate
                self.publisher.publish(vel_msg)

                # Check if goal is reached
                if calculate_linear_error(current_pose, target_object) < self.reachThreshold:
                    print("Reached object.")
                    self.publisher.publish(Twist())  # Stop robot
                    self.state_machine.pick_object()

        elif self.state_machine.state == "PICK":
            print("State: PICK")
            # Send commands to STM32 to pick up the object
            success = self.pick_up_object()

            if success:
                print("Object picked up.")
                # Remove object from queue to prevent double servicing
                del self.object_queue[0]
                # Find tidy destination - hardcoded for now
                x_dropoff, y_dropoff, z_dropoff = self.drop_off_locations[self.current_drop_off_index]
                print(f"Assigned drop-off location: ({x_dropoff}, {y_dropoff}, {z_dropoff})")
                self.goal = (x_dropoff, y_dropoff)
                #update dropoff index for next object
                self.current_drop_off_index += 1

                self.state_machine.plan_path_to_destination()

        elif self.state_machine.state == "TO_TIDY_DESTINATION":
            print("State: TO_TIDY_DESTINATION")
            
            path_to_destination = self.planner.plan(current_pose[:2], self.goal)
            
            if path_to_destination:
                velocity, yaw_rate = self.controller.vel_request(current_pose, path_to_destination)
                vel_msg = Twist()
                vel_msg.linear.x = velocity
                vel_msg.angular.z = yaw_rate
                self.publisher.publish(vel_msg)

                if calculate_linear_error(current_pose, self.goal) < self.reachThreshold:
                    print("Reached tidy destination.")
                    self.publisher.publish(Twist())  # Stop robot
                    self.state_machine.place_object()

        elif self.state_machine.state == "PLACE_OBJECT":
            print("State: PLACE_OBJECT")
            
            success = self.place_object()
            
            if success:
                print("Object placed successfully.")
                
                if not len(self.object_queue): 
                    return 



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
    
    if args.motion == "point":
        DM=decision_maker(Twist, "/cmd_vel", 10, motion_type=POINT_PLANNER)
    elif args.motion == "trajectory":
        DM=decision_maker(Twist, "/cmd_vel", 10, motion_type=TRAJECTORY_PLANNER)
    else:
        print("invalid motion type", file=sys.stderr)



    try:
        spin(DM)
    except SystemExit:
        print(f"reached there successfully {DM.localizer.pose}")




if __name__=="__main__":
    argParser=argparse.ArgumentParser(description="point or trajectory") 
    argParser.add_argument("--motion", type=str, default="trajectory")
    args = argParser.parse_args()

    main(args)
