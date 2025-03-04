
from mapUtilities import *
from a_star import *

POINT_PLANNER=0; TRAJECTORY_PLANNER=1

class planner:
    def __init__(self, type_, mapName="room"):

        self.type=type_
        self.mapName=mapName

    
    def plan(self, startPose, endPose):

        print(f"Type of planner {self.type} \n\n")
        
        if self.type==POINT_PLANNER:
            return self.point_planner(endPose)
        
        elif self.type==TRAJECTORY_PLANNER:
            self.costMap=None
            self.initTrajectoryPlanner()
            return self.trajectory_planner(startPose, endPose)


    def point_planner(self, endPose):
        return endPose

    def initTrajectoryPlanner(self):


        # TODO PART 5 Create the cost-map, the laser_sig is 
        # the standard deviation for the gausiian for which
        # the mean is located on the occupant grid. 
        self.m_utilites=mapManipulator(laser_sig=0.5) #kept as default for now
            
        self.costMap=self.m_utilites.make_likelihood_field()
        

    def trajectory_planner(self, startPoseCart, endPoseCart):


        # This is to convert the cartesian coordinates into the 
        # the pixel coordinates of the map image, remmember,
        # the cost-map is in pixels. You can by the way, convert the pixels
        # to the cartesian coordinates and work by that index, the a_star finds
        # the path regardless. 
        startPose=self.m_utilites.position_2_cell(startPoseCart)
        endPose=self.m_utilites.position_2_cell(endPoseCart)

        # # Debugging: Print the map boundaries and positions
        # #------------------------------------------------------
        # print(f"CostMap shape: {self.costMap.shape}")
        # print(f"Start position (cartesian): {startPoseCart}, Start position (cell): {startPose}")
        # print(f"End position (cartesian): {endPoseCart}, End position (cell): {endPose}")
        
        # #Debugging:
        # max_rows, max_cols = self.costMap.shape
        # if not (0 <= startPose[0] < max_rows and 0 <= startPose[1] < max_cols):
        #     print(f"Error: Start position {startPose} is out of bounds!")
        #     return []
        # if not (0 <= endPose[0] < max_rows and 0 <= endPose[1] < max_cols):
        #     print(f"Error: End position {endPose} is out of bounds!")
        #     return []

        # # Ensure start and end positions are not in obstacles
        # if self.costMap[startPose[0], startPose[1]] > 0.8:
        #     print(f"Start position {startPose} is in an obstacle!")
        #     return []
        # if self.costMap[endPose[0], endPose[1]] > 0.8:
        #     print(f"End position {endPose} is in an obstacle!")
        #     return []
        # #-------------------------------------------------------

        # TODO PART 5 convert the cell pixels into the cartesian coordinates
        #Find path using a_start algorithm
        # print(f"CostMap {self.costMap}\n")

        print(f"startPose {startPose}, endPose {endPose}\n")

        path = search(self.costMap, startPose, endPose)
        # TODO PART 5 convert the cell pixels into the cartesian coordinates
        Path = list(map(self.m_utilites.cell_2_position, path))


        # TODO PART 5 return the path as list of [x,y]
        return Path




if __name__=="__main__":

    m_utilites=mapManipulator()
    
    map_likelihood=m_utilites.make_likelihood_field()

    # you can use this part of the code to test your 
    # search algorithm regardless of the ros2 hassles
    
