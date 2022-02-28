################################
# Name: Shivam Haldiya
# SID:  56041529
# EID:  Shaldiya2
#
# Instructions:
# The code is organised in blocks.
# When you finish running a block, put it in comments it again
# Only Remove a block from Comment when you want to run it.
#
# For question 1-3:
# Uncomment the commen lines 151-186 AND 227-233
#
# Question 1: Uncomment line 198
# Question 2: Uncomment line 211
# Question 3: Uncomment line 224
# Question 3: Uncomment lines 236-339
################################

import argparse
import time
import sys
import msgpack
import numpy as np
import networkx as nx
from enum import Enum, auto
from planning_utils import *
from udacidrone import Drone
from udacidrone.connection import MavlinkConnection
from udacidrone.messaging import MsgID
from udacidrone.frame_utils import global_to_local
import matplotlib as mpl
# Used for mac compatibility for plotting
# If creating any issues, kindly delete this line
mpl.use('tkagg')
import matplotlib.pyplot as plt
import time

class States(Enum):
    MANUAL = auto()
    ARMING = auto()
    TAKEOFF = auto()
    WAYPOINT = auto()
    LANDING = auto()
    DISARMING = auto()
    PLANNING = auto()


class MotionPlanning(Drone):

    def __init__(self, connection):
        super().__init__(connection)

        self.target_position = np.array([0.0, 0.0, 0.0])
        self.waypoints = []
        self.in_mission = True
        self.check_state = {}

        # initial state
        self.flight_state = States.MANUAL

        # register all your callbacks here
        self.register_callback(MsgID.LOCAL_POSITION, self.local_position_callback)
        self.register_callback(MsgID.LOCAL_VELOCITY, self.velocity_callback)
        self.register_callback(MsgID.STATE, self.state_callback)

    def local_position_callback(self):
        if self.flight_state == States.TAKEOFF:
            if -1.0 * self.local_position[2] > 0.95 * self.target_position[2]:
                self.waypoint_transition()
        elif self.flight_state == States.WAYPOINT:
            if np.linalg.norm(self.target_position[0:2] - self.local_position[0:2]) < 1.0:
                if len(self.waypoints) > 0:
                    self.waypoint_transition()
                else:
                    if np.linalg.norm(self.local_velocity[0:2]) < 1.0:
                        self.landing_transition()

    def velocity_callback(self):
        if self.flight_state == States.LANDING:
            if self.global_position[2] - self.global_home[2] < 0.1:
                if abs(self.local_position[2]) < 0.01:
                    self.disarming_transition()

    def state_callback(self):
        if self.in_mission:
            if self.flight_state == States.MANUAL:
                self.arming_transition()
            elif self.flight_state == States.ARMING:
                if self.armed:
                    self.plan_path()
            elif self.flight_state == States.PLANNING:
                self.takeoff_transition()
            elif self.flight_state == States.DISARMING:
                if ~self.armed & ~self.guided:
                    self.manual_transition()

    def arming_transition(self):
        self.flight_state = States.ARMING
        print("arming transition")
        self.arm()
        self.take_control()

    def takeoff_transition(self):
        self.flight_state = States.TAKEOFF
        print("takeoff transition")
        self.takeoff(self.target_position[2])

    def waypoint_transition(self):
        self.flight_state = States.WAYPOINT
        print("waypoint transition")
        self.target_position = self.waypoints.pop(0)
        print('target position', self.target_position)
        self.cmd_position(self.target_position[0], self.target_position[1], self.target_position[2], self.target_position[3])

    def landing_transition(self):
        self.flight_state = States.LANDING
        print("landing transition")
        self.land()

    def disarming_transition(self):
        self.flight_state = States.DISARMING
        print("disarm transition")
        self.disarm()
        self.release_control()

    def manual_transition(self):
        self.flight_state = States.MANUAL
        print("manual transition")
        self.stop()
        self.in_mission = False

    def send_waypoints(self):
        print("Sending waypoints to simulator ...")
        data = msgpack.dumps(self.waypoints)
        self.connection._master.write(data)

    def plan_path(self):
    
        self.flight_state = States.PLANNING
        print("Searching for a path ...")
        TARGET_ALTITUDE = 5
        SAFETY_DISTANCE = 5

        self.target_position[2] = TARGET_ALTITUDE


        


################################## QUESTION (1-3) ####################################
#        #TODO: UNCOMMENT THIS BLOCK TO CHECK ANSWERS 1-3
#        print('global home {0}, position {1}, local position {2}'.format(self.global_home, self.global_position,self.local_position))
#        #Read in obstacle map
#        data = np.loadtxt('colliders.csv', delimiter=',', dtype='Float64', skiprows=2)
#
#        #Define a grid for a particular altitude and safety margin around obstacles
#        grid, north_offset, east_offset = create_grid(data, TARGET_ALTITUDE, SAFETY_DISTANCE)
#        print("North offset = {0}, east offset = {1}".format(north_offset, east_offset))
#        #Define starting point on the grid (this is just grid center)
#        grid_start = (-north_offset, -east_offset)
#        #TODO: convert start position to current position rather than map center
#        grid_point = []
#        grid_point.append((-north_offset - 2, -east_offset + 20))
#        grid_point.append((-north_offset + 4, -east_offset + 15))
#        grid_point.append((-north_offset + 8, -east_offset + 30))
#        #        grid_point.append((-north_offset + 8, -east_offset + 8))
#        #        grid_point.append((-north_offset + 5, -east_offset + 5))
#        #        grid_point.append((-north_offset+ 2, -east_offset + 2))
#
#        #Set goal as some arbitrary position on the grid
#        grid_goal = (-north_offset + 10, -east_offset + 10)
#        #TODO: adapt to set goal as latitude / longitude position and convert
#
#         #Run A* to find a path from start to goal
#         #TODO: add diagonal motions with a cost of sqrt(2) to your A* implementation
#         #or move to a different search space such as a graph (not done here)
#        print('Local Start and Goal: ', grid_start, grid_goal)
#
#        ## a_star Search Commented out for testing iterative_astar Search
#        ## path, dist = a_star(grid, heuristic, grid_start, grid_goal)
#        #path, dist = a_star(grid, manhattan_heurestic, grid_start, grid_goal)
#
#         #TODO: prune path to minimize number of waypoints
#         #TODO (if you're feeling ambitious): Try a different approach altogether!
######################################################################################
        
        ########################## QUESTION 1 ########################################
        ##  Iterative A* Algorithm
        ##  Question 1 (20 points): Implementing the iterative deepening A* search algorithm
        ##
        ## You are expected to write an iterative deepening A* search algorithm in
        ## planning_utils.py named iterative_astar(grid, h, start, goal) to help the drone plan
        ## routes. Procedure for the iterative deepening A* search algorithm can be found in the
        ## lecture slides.
        ##
        ## TODO: UNCOMMENT AND PROPERLY INDENT THE LINE BELOW TO CHECK ANSWER 1
        ##path, dist = iterative_astar(grid, heuristic, grid_start, grid_goal)
        ##############################################################################
        
        
        ########################## QUESTION 2 ########################################
        ## Question 2 (20 points): Implementing the uniform cost search algorithm
        ##
        ## You are expected to write a uniform cost search algorithm in planning_utils.py named
        ## ucs(grid, h, start, goal) to help the drone plan routes. Procedure for the uniform
        ## cost search algorithm can be found in the lecture slides. Note that you should first
        ## of all design and implement the cost function.
        ##
        ## TODO: UNCOMMENT AND PROPERLY INDENT THE LINE BELOW TO CHECK ANSWER 2
        ##path, dist = ucs(grid, manhattan_heurestic, grid_start, grid_goal)
        ## print(dist)
        ##############################################################################
        
        ########################## QUESTION 3 ########################################
        ## Question 3 (20 points): Implementing different heuristics for A* and the A*
        ## search for traversing 3 fixed points. In the current A* version in planning_utils.py, the
        ## Manhatten distance is used as the heuristic. You are encouraged to propose one valid heuristic,
        ## implement that, and see how the planned routes change. You are also required to set three fixed
        ## points in motion_planning.py which the drone has to traverse before reaching to the destination.
        ## Building on this, you re-implement the A* search algorithm in order to go through the 3 points.
        ##
        ## TODO: UNCOMMENT AND PROPERLY INDENT THE LINE BELOW TO CHECK ANSWER 3
        ##path, dist = threePoint_a_star(grid, manhattan_heurestic, grid_start, grid_goal, grid_point)
        ##############################################################################
        
        ########################## QUESTION 1-3 ######################################
        ## TODO: UNCOMMENT THIS BLOCK TO CHECK ANSWERS 1-3
        ##
        ## waypoints = [[p[0] + north_offset, p[1] + east_offset, TARGET_ALTITUDE, 0] for p in path]
        ## self.waypoints = waypoints
        ## self.send_waypoints()
        ##############################################################################
        
        
############################################ QUESTION 4 ##############################
#        # TODO: UNCOMMENT THIS BLOCK TO CHECK ANSWER 4
#        # !!!: This code does NOT generate the lines with the green dots but you can see the drone moving
#        # Question 4 (20 points): Implementing the breadth-first search algorithm with graph search
#        # Based on the graph G, now you are ready to implement the breadth-first search algorithm to search
#        # the graph and return the path.
#
#        colliders_file = 'colliders.csv'
#        lat0, lon0 = read_home(colliders_file)
#        print(f'Home lat : {lat0}, lon : {lon0}')
#        self.set_home_position(lon0, lat0, 0)
#        local_north, local_east, local_down = global_to_local(self.global_position, self.global_home)
#        print(f'Local => north : {local_north}, east : {local_east}, down : {local_down}')
#        print('global home {0}, position {1}, local position {2}'.format(self.global_home, self.global_position, self.local_position))
#
#        # Read in obstacle map
#        data = np.loadtxt(colliders_file, delimiter=',', dtype='Float64', skiprows=3)
#
#        # Define a grid for a particular altitude and safety margin around obstacles
#        grid, north_offset, east_offset = create_grid(data, TARGET_ALTITUDE, SAFETY_DISTANCE)
#        print("North offset = {0}, east offset = {1}".format(north_offset, east_offset))
#        north_offset = int(np.ceil(local_north - north_offset))
#        east_offset = int(np.ceil(local_east - east_offset))
#        grid_start = (-north_offset, -east_offset)
#
#        data = np.loadtxt('colliders.csv', delimiter=',', dtype='Float64', skiprows=2)
#
#        # Define a flying altitude (feel free to change this)
#        drone_altitude = 5
#        safety_distance = 3
#
#        grid, edges = create_grid_and_edges(data, drone_altitude, safety_distance)
##        print('Found %5d edges' % len(edges))
#
#        plt.imshow(grid, origin='lower', cmap='Greys')
#
#        ## Creating a graph
#        G = nx.Graph()
#
#        # Stepping through each edge
#        for e in edges:
#            p1 = e[0]
#            p2 = e[1]
##            plt.plot([p1[1], p2[1]], [p1[0], p2[0]], 'b-')
#            ## Adding the edge to the graph along with its weight
#            dist = np.linalg.norm(np.array(p2) - np.array(p1))
#            G.add_edge(p1, p2, weight=dist)
#
##        plt.xlabel('EAST')
##        plt.ylabel('NORTH')
##        #print("Done Plotting")
##        plt.show()
#
#        print("-- Starting BFS --")
#        path = bfs(G, edges[500][0], edges[520][0])
#        print("-- BFS Finished --")
#
#        plt.clf()
#        plt.imshow(grid, origin='lower', cmap='Greys')
#        tmp = 0
#        for i in range(0, len(path)-1):
#            p1 = path[i]
#            p2 = path[i+1]
#            plt.plot([p1[1], p2[1]], [p1[0], p2[0]], 'b-')
#        plt.xlabel('EAST')
#        plt.ylabel('NORTH')
#        plt.show()
#
#
##        p,_ = a_star(grid, manhattan_heurestic, grid_start, (int(path[0][0]), int(path[0][1])))
#        p = []
#        d, tmp = 0, 0
#
#        visited = {point: False for point in path}
#        for i in range(0, len(path)):
#            try:
#                p1, d1 = a_star(grid, manhattan_heurestic, (int(path[i][0]), int(path[i][1])), (int(path[i+1+tmp][0]), int(path[i+1+tmp][1])))
#                visited[i] = True
#                p = p+p1
#                d = d+d1
#                tmp = 0
#            except:
#                if i!=0 or i!=len(path)-1 or visited[path[i-1]]!=False:
#                    i = i-1
#                    tmp = tmp+1
#
#        print("Path for the shown graph Calculated")
#        # Convert path to waypoints
#        waypoints = [[p[0] - north_offset, p[1] - east_offset, drone_altitude, 0] for p in path]
#        self.send_waypoints()
#        self.waypoints = waypoints
#######################################################################################
        
    def start(self):
        self.start_log("Logs", "NavLog.txt")

        print("starting connection")
        self.connection.start()

        # Only required if they do threaded
        # while self.in_mission:
        #    pass

        self.stop_log()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', type=int, default=5760, help='Port number')
    parser.add_argument('--host', type=str, default='127.0.0.1', help="host address, i.e. '127.0.0.1'")
    args = parser.parse_args()

    conn = MavlinkConnection('tcp:{0}:{1}'.format(args.host, args.port), timeout=120)
    drone = MotionPlanning(conn)
    time.sleep(1)

    drone.start()
