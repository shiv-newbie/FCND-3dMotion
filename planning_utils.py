################################
# Name: Shivam Haldiya
# SID:  56041529
# EID:  Shaldiya2
################################

from enum import Enum
from queue import PriorityQueue
import numpy as np
from scipy.spatial import Voronoi
from bresenham import bresenham
import re

def read_home(filename):
    """
    Reads home (lat, lon) from the first line of the `file`.
    """
    with open(filename) as f:
        first_line = f.readline()
    match = re.match(r'^lat0 (.*), lon0 (.*)$', first_line)
    if match:
        lat = match.group(1)
        lon = match.group(2)
    return np.fromstring(f'{lat},{lon}', dtype='Float64', sep=',')

# Here you'll modify the `create_grid()` method from a previous exercise # In this new function you'll record obstacle centres and
# create a Voronoi graph around those points
def create_grid_and_edges(data, drone_altitude, safety_distance):
    """
    Returns a grid representation of a 2D configuration space
     along with Voronoi graph edges given obstacle data and the drone's altitude.
    """
    # minimum and maximum north coordinates
    north_min = np.floor(np.min(data[:, 0] - data[:, 3]))
    north_max = np.ceil(np.max(data[:, 0] + data[:, 3]))
    
    # minimum and maximum east coordinates
    east_min = np.floor(np.min(data[:, 1] - data[:, 4]))
    east_max = np.ceil(np.max(data[:, 1] + data[:, 4]))
    
    # given the minimum and maximum coordinates we can # calculate the size of the grid.
    north_size = int(np.ceil((north_max - north_min)))
    east_size = int(np.ceil((east_max - east_min)))
    
    # Initialize an empty grid
    grid = np.zeros((north_size, east_size))
    
    # Center offset for grid
    north_min_center = np.min(data[:, 0])
    east_min_center = np.min(data[:, 1])
    
    # Define a list to hold Voronoi points
    points = []
    # Populate the grid with obstacles
    for i in range(data.shape[0]):
        north, east, alt, d_north, d_east, d_alt = data[i, :]
        if alt + d_alt + safety_distance > drone_altitude:
            obstacle = [
                int(north - d_north - safety_distance - north_min_center),
                int(north + d_north + safety_distance - north_min_center),
                int(east - d_east - safety_distance - east_min_center),
                int(east + d_east + safety_distance - east_min_center),
            ]
            grid[obstacle[0]:obstacle[1], obstacle[2]:obstacle[3]] = 1
        
            # add center of obstacles to points list
            points.append([north - north_min, east - east_min])
        
    # TODO: create a voronoi graph based on
    # location of obstacle centres
    graph = Voronoi(points)
    # TODO: check each edge from graph.ridge_vertices for collision
    edges = []
    for edge in graph.ridge_vertices:
        point1 = graph.vertices[edge[0]]
        point2 = graph.vertices[edge[1]]
        
        cells = list(bresenham(int(point1[0]), int(point1[1]), int(point2 [0]), int(point2[1])))
        infeasible = False

        for cell in cells:
            if np.amin(cell) < 0 or cell[0] >= grid.shape[0] or cell[1] >= grid.shape[1]:
                infeasible = True
                break
            if grid[cell[0], cell[1]] == 1:
                infeasible = True
                break
        if infeasible == False:
            point1 = (point1[0], point1[1])
            point2 = (point2[0], point2[1])
            edges.append((point1,point2))
    return grid, edges
    
def create_grid(data, drone_altitude, safety_distance):
    """
    Returns a grid representation of a 2D configuration space
    based on given obstacle data, drone altitude and safety distance
    arguments.
    """

    # minimum and maximum north coordinates
    north_min = np.floor(np.min(data[:, 0] - data[:, 3]))
    north_max = np.ceil(np.max(data[:, 0] + data[:, 3]))

    # minimum and maximum east coordinates
    east_min = np.floor(np.min(data[:, 1] - data[:, 4]))
    east_max = np.ceil(np.max(data[:, 1] + data[:, 4]))

    # given the minimum and maximum coordinates we can
    # calculate the size of the grid.
    north_size = int(np.ceil(north_max - north_min))
    east_size = int(np.ceil(east_max - east_min))

    # Initialize an empty grid
    grid = np.zeros((north_size, east_size))

    # Populate the grid with obstacles
    for i in range(data.shape[0]):
        north, east, alt, d_north, d_east, d_alt = data[i, :]
        if alt + d_alt + safety_distance > drone_altitude:
            obstacle = [
                int(np.clip(north - d_north - safety_distance - north_min, 0, north_size-1)),
                int(np.clip(north + d_north + safety_distance - north_min, 0, north_size-1)),
                int(np.clip(east - d_east - safety_distance - east_min, 0, east_size-1)),
                int(np.clip(east + d_east + safety_distance - east_min, 0, east_size-1)),
            ]
            grid[obstacle[0]:obstacle[1]+1, obstacle[2]:obstacle[3]+1] = 1

    return grid, int(north_min), int(east_min)


# Assume all actions cost the same.
class Action(Enum):
    """
    An action is represented by a 3 element tuple.

    The first 2 values are the delta of the action relative
    to the current grid position. The third and final value
    is the cost of performing the action.
    """

    WEST = (0, -1, 1)
    EAST = (0, 1, 1)
    NORTH = (-1, 0, 1)
    SOUTH = (1, 0, 1)

    @property
    def cost(self):
        return self.value[2]

    @property
    def delta(self):
        return (self.value[0], self.value[1])


def valid_actions(grid, current_node):
    """
    Returns a list of valid actions given a grid and current node.
    """
    valid_actions = list(Action)
    n, m = grid.shape[0] - 1, grid.shape[1] - 1
    x, y = current_node

    # check if the node is off the grid or
    # it's an obstacle

    if x - 1 < 0 or grid[x - 1, y] == 1:
        valid_actions.remove(Action.NORTH)
    if x + 1 > n or grid[x + 1, y] == 1:
        valid_actions.remove(Action.SOUTH)
    if y - 1 < 0 or grid[x, y - 1] == 1:
        valid_actions.remove(Action.WEST)
    if y + 1 > m or grid[x, y + 1] == 1:
        valid_actions.remove(Action.EAST)

    return valid_actions


def a_star(grid, h, start, goal):
    path = []
    path_cost = 0
    queue = PriorityQueue()
    queue.put((0, start))
    visited = set(start)

    branch = {}
    found = False
    
    while not queue.empty():
        item = queue.get()
        current_node = item[1]
        if current_node == start:
            current_cost = 0.0
        else:              
            current_cost = branch[current_node][0]
            
        if current_node == goal:        
#            print('Found a path.')
            found = True
            break
        else:
            for action in valid_actions(grid, current_node):
                # get the tuple representation
                da = action.delta                   # South, West, East, North
                next_node = (current_node[0] + da[0], current_node[1] + da[1])
                branch_cost = current_cost + action.cost
                queue_cost = branch_cost + h(next_node, goal)
                
                if next_node not in visited:                
                    visited.add(next_node)               
                    branch[next_node] = (branch_cost, current_node, action)
                    queue.put((queue_cost, next_node))
             
    if found:
        # retrace steps
        n = goal
        path_cost = branch[n][0]
        path.append(goal)
        while branch[n][1] != start:
            path.append(branch[n][1])
            n = branch[n][1]
        path.append(branch[n][1])
    else:
        print('**********************')
        print('Failed to find a path!')
        print('**********************') 
    return path[::-1], path_cost


## Isn't this eulclidian distance?
def heuristic(position, goal_position):
    return np.linalg.norm(np.array(position) - np.array(goal_position))
    
## Q3 -> Valid heuristic
def manhattan_heurestic(position, goal_position):
    return np.abs(np.array(position) - np.array(goal_position)).sum()

def iterative_astar(grid, h, start, goal):
    print("\n--- Iterative A* Search Started ---")
    x = 0
    path = []
    thold = h(start, goal)                  # Finding out the threshhold for starting
    while True:                             # Iterating with different threshholds for different levels
        x = x+1
#        DEBUGGING STATEMENT
#        print("Iteration{0}: Threshold = {1}".format(x, thold))
        visited = set(start)
        dist, p = iterative_astar_helper(grid, h, start, goal, 0, thold, visited, path)
#        DEBUGGING STATEMENT
#        print("Distance = {0}".format(dist))
        ## If there is no breach and we haven't found our goal node,
        ## it means that we have failed to find a path!
        if dist == float("inf"):
            print('**********************')
            print('Failed to find a path!')
            print('**********************')
            print("--- Iterative A* Search Ended ---\n")
            return [], 0
        ## Found the node
        elif dist < 0:
            path.append(start)
            print('Found a path.')
            print("--- Iterative A* Search Ended ---\n")
            return p[::-1], -dist
        ## We want to set the threshold as the minimum of all the
        ## breaches of thresholds we have found
        else:
            thold = dist

def iterative_astar_helper(grid, h, position, goal, dist, thold, visited, path):
    
#   DEBUGGING STATEMENT
#    print("Position = ({0}, {1}) \t Goal = ({2}, {3})".format(position[0], position[1], goal[0], goal[1]))
    
    ## If we have reached the goal state
    if position == goal:
        return (-dist, path)
    
    ## Checking if the distance with heuristics breach the threshold
    checkBreach = dist + h(position, goal)
    if checkBreach > thold:
#   DEBUGGING STATEMENT
#        print("*** ALERT - Hit Threshold ***")
        return (checkBreach, path)
    
    ## Setting the minimum value to infinity for further changes
    min = float("inf")
    
    ## for actions in North, West, East, South
    for action in valid_actions(grid, position):
        da = action.delta
        
        ## getting the location and cost for the next node
        next_node = (position[0] + da[0], position[1] + da[1])
        branch_cost = dist + action.cost
        
        ## We do not want to visit the parent node. Hence, checking
        ## if the node is already visited or not
        if next_node not in visited:
            
            ## Adding the node to visited nodes
            visited.add(next_node)
            
#            DEBUGGING STATEMENT
#            print("Position = ({0}, {1}) \t Going to -> ({2}, {3})".format(position[0], position[1], da[0], da[1]))

            ## Expanding the node
            tmp_dist, path = iterative_astar_helper(grid, h, next_node, goal, branch_cost, thold, visited, path)
            
            ## If node is found
            if tmp_dist < 0:
                path.append(next_node)
                return (tmp_dist, path)
                
            ## If there is a breach of threshold, we keep the minimum
            ## breach as the next threshold
            elif tmp_dist < min:
                min = tmp_dist
               
#   DEBUGGING STATEMENT
#    print("MIN: {0}".format(min))
    
    return min, path

## TODO: Check if this is correct cus I dunno if it is
def ucs(grid, h, start, goal):
    path = []
    path_cost = 0
    queue = PriorityQueue()
    queue.put((0, start))
    visited = set(start)

    branch = {}
    found = False
    
    while not queue.empty():
        item = queue.get()
        current_node = item[1]
        if current_node == start:
            current_cost = 0.0
        else:
            current_cost = branch[current_node][0]
            
        if current_node == goal:
            print('Found a path.')
            found = True
            break
        else:
            for action in valid_actions(grid, current_node):
                # get the tuple representation
                da = action.delta                   # South, West, East, North
                next_node = (current_node[0] + da[0], current_node[1] + da[1])
                branch_cost = current_cost + action.cost
                queue_cost = h(next_node, goal)
                
                if next_node not in visited:
                    visited.add(next_node)
                    branch[next_node] = (branch_cost, current_node, action)
                    queue.put((queue_cost, next_node))
             
    if found:
        # retrace steps
        n = goal
        path_cost = branch[n][0]
        path.append(goal)
        while branch[n][1] != start:
            path.append(branch[n][1])
            n = branch[n][1]
        path.append(branch[n][1])
    else:
        print('**********************')
        print('Failed to find a path!')
        print('**********************')
    return path[::-1], path_cost

#
#class matrix:
#    def __init__(self, points: int):
#        self.points = points
#        self.adj = [[] for _ in range(V)]
#
#
#    def addEdgeRev(self, u: int, v: int, w: int):
#        self.adj[v].append((u, w))
#
#
#    def shortestPath(self, dest: int):
#



def threePoint_a_star(grid, h, start, goal, points):
    path = []
    min = float("inf")
    for i in range(0, len(points)):
        for j in range(0, len(points)):
            if i == j:
                continue
            tmp = h(start, points[i]) + h(points[i], points[j]) + h(points[j], points[len(points)-i-j]) + h(points[len(points)-i-j], goal)
            if tmp < min:
                min = tmp
                path = [i, j, len(points)-i-j]
    
#    print(path)
    p,d = a_star(grid, h, start, points[path[0]])
    print("Path to point 1 calculated")
    for i in range(0, len(path)-1):
        print("Path to point {0} calculated".format(i+2))
        p1,d1 = a_star(grid, h, points[path[i]], points[path[i+1]])
        p = p + p1
        d = d + d1
    p1,d1 = a_star(grid, h, points[path[len(path)-1]], goal)
    p = p + p1
    d = d + d1
    print("Path to Goal calculated. Let's Go!")
    
    
    return p, d

def bfs(graph, StartNode, goalNode):
    visited = {node: False for node in graph.nodes}
    queue = [StartNode]
    visited[StartNode] = True
    path = []
    branch = {}
    found = False
    
    while queue:
        cur_node = queue.pop(0)
        if cur_node == goalNode:
            found = True
            break
        
        for node in graph.neighbors(cur_node):
            if not visited[node]:
                branch[node] = cur_node
                queue.append(node)
                visited[node] = True
    if found:
        # retrace steps
        n = goalNode
        path.append(goalNode)
        while branch[n] != StartNode:
            path.append(branch[n])
            n = branch[n]
        path.append(branch[n])
    else:
        print('**********************')
        print('Failed to find a path!')
        print('**********************')
        
#    print("Result: ", "->".join(str(v) for v in path))
    return path[::-1]
    
