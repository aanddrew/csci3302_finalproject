"""lab5 controller."""
from controller import Robot, Motor, Camera, RangeFinder, Lidar, Keyboard
import math
import numpy as np
from matplotlib import pyplot as plt
from scipy.signal import convolve2d # Uncomment if you want to use something else for finding the configuration space

MAX_SPEED = .75*7.0  # [rad/s]
MAX_SPEED_MS = 0.633 # [m/s]
AXLE_LENGTH = 0.50 # m
WHEEL_RADIUS = 0.1 # m
MOTOR_LEFT = 10
MOTOR_RIGHT = 11
N_PARTS = 12

LIDAR_ANGLE_BINS = 667
LIDAR_SENSOR_MAX_RANGE = 2.75 # Meters
LIDAR_ANGLE_RANGE = math.radians(240)


##### vvv [Begin] Do Not Modify vvv #####

# create the Robot instance.
robot = Robot()
# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())
DELTA_TIME = 1/timestep;


# The Tiago robot has multiple motors, each identified by their names below
part_names = ("head_2_joint", "head_1_joint", "torso_lift_joint", "arm_1_joint",
              "arm_2_joint",  "arm_3_joint",  "arm_4_joint",      "arm_5_joint",
              "arm_6_joint",  "arm_7_joint",  "wheel_left_joint", "wheel_right_joint")

# All motors except the wheels are controlled by position control. The wheels
# are controlled by a velocity controller. We therefore set their position to infinite.
target_pos = (0.0, 0.0, 0.09, 0.07, 1.02, -3.16, 1.27, 1.32, 0.0, 1.41, 'inf', 'inf')
robot_parts=[]

for i in range(N_PARTS):
    robot_parts.append(robot.getDevice(part_names[i]))
    robot_parts[i].setPosition(float(target_pos[i]))
    robot_parts[i].setVelocity(robot_parts[i].getMaxVelocity() / 2.0)

# The Tiago robot has a couple more sensors than the e-Puck
# Some of them are mentioned below. We will use its LiDAR for Lab 5

# range = robot.getDevice('range-finder')
# range.enable(timestep)
# camera = robot.getDevice('camera')
# camera.enable(timestep)
# camera.recognitionEnable(timestep)
lidar = robot.getDevice('Hokuyo URG-04LX-UG01')
lidar.enable(timestep)
lidar.enablePointCloud()

# We are using a GPS and compass to disentangle mapping and localization
gps = robot.getDevice("gps")
gps.enable(timestep)
compass = robot.getDevice("compass")
compass.enable(timestep)

# We are using a keyboard to remote control the robot
keyboard = robot.getKeyboard()
keyboard.enable(timestep)

# The display is used to display the map. We are using 360x360 pixels to
# map the 12x12m2 apartment
display = robot.getDevice("display")

# Odometry
pose_x     = 0
pose_y     = 0
pose_theta = 0

vL = 0
vR = 0

lidar_sensor_readings = [] # List to hold sensor readings
lidar_offsets = np.linspace(-LIDAR_ANGLE_RANGE/2., +LIDAR_ANGLE_RANGE/2., LIDAR_ANGLE_BINS)
lidar_offsets = lidar_offsets[83:len(lidar_offsets)-83] # Only keep lidar readings not blocked by robot chassis

map = None
##### ^^^ [End] Do Not Modify ^^^ #####
def getConfigSpace (map, radius = 10, threshHold = 0.5):
  shape  = map.shape;
  newMap = np.zeros(shape)
  print('shape', shape)
  for i in range(shape[0]):
    for j in range(shape[1]):
      #print(i)
      if map[i][j] > threshHold:
        left = j-radius
        right = j+radius
        lower = i+radius
        upper = i-radius
        
        if left<0:
          left = 0
        if right>shape[1]-1:
          right = shape[1]-1
        if lower > shape[0]-1:
          lower = shape[0]-1
        if upper < 0:
          upper = 0
        for k in range(upper,lower):
            for n in range(left,right):
                newMap[k][n] = 1
  return newMap

def displayMap(map):
  for disp_x in range(360):
    for disp_y in range(360):
        map[disp_x][disp_y] += 0.005
        if disp_x < 0 or disp_x >= 360 or disp_y < 0 or disp_y >= 360:
            continue

        if map[disp_x][disp_y] > 0.99:
            map[disp_x][disp_y] = 0.99

            g = map[disp_x][disp_y]
            byte = int(g * 255)
            byte2 = byte * 256
            byte3 = byte2 * 256

            color = byte + byte2 + byte3
            #print(hex(color))
            #if map[disp_x][disp_y] > 1:
            display.setColor(color)
            display.drawPixel(disp_x, disp_y)

def world_to_robot(waypoint,start_position_global,theta):
    dx = waypoint[0] - start_position_global[0]
    dz = waypoint[1] - start_position_global[1]
    dx = dx*(np.cos(theta)+np.sin(theta))
    dz = dz*(-np.sin(theta)+np.cos(theta))
    return (dx,-dz)
##################### IMPORTANT #####################
# Set the mode here. Please change to 'autonomous' before submission
mode = 'manual' # Part 1.1: manual mode
# mode = 'planner'
# mode = 'autonomous'


sqrt2 = np.sqrt(2)

def get_neighbors(map, node):
    potential_neighbors = []
    for i in [-1,0,1]:
        for j in [-1,0,1]:
            if i == 0 and j == 0:
                continue
            if node[0] + i < 0 or node[0] + i >= 360 or node[1] + j < 0 or node[1] + j >= 360:
                continue
            if map[node[0] + i][node[1] + j] > 0.9:
                continue
            neighbor = (node[0] + i, node[1] + j)
            
            dist = 1
            if abs(i) == 1 and abs(j) == 1:
                dist = sqrt2
            
            potential_neighbors.append((neighbor, dist))
    return potential_neighbors
            
    

def euclid(x, y):
    return np.sqrt((x[0] - y[0])**2 + (x[1] - y[1]) **2)
	
	
def draw_rect(x, y, radius, color):
    for disp_x in range(x - radius, x + radius):
        for disp_y in range(y - radius, y + radius):
            if disp_x < 0 or disp_x >= 360 or disp_y < 0 or disp_y >= 360:
                continue
                
            #byte = int(g * 255)
            #byte2 = byte * 256
            #byte3 = byte2 * 256
            
            #color = byte + byte2 + byte3
            #print(hex(color))

            #if map[disp_x][disp_y] > 1:
            display.setColor(color)
            display.drawPixel(disp_x, disp_y)

draw_rect(0, 0, 60, 0xffffff)
###################
#
# Planner
#
###################
if mode == 'planner':
    # Part 2.3: Provide start and end in world coordinate frame and convert it to map's frame
    start_w = None # (Pose_X, Pose_Z) in meters
    end_w = None # (Pose_X, Pose_Z) in meters

    # Convert the start_w and end_w from the webots coordinate frame into the map frame
    start = None # (x, y) in 360x360 map
    end = None # (x, y) in 360x360 map

    # Part 2.3: Implement A* or Dijkstra's Algorithm to find a path
    def path_planner(map, start, end):
        '''
        :param map: A 2D numpy array of size 360x360 representing the world's cspace with 0 as free space and 1 as obstacle
        :param start: A tuple of indices representing the start cell in the map
        :param end: A tuple of indices representing the end cell in the map
        :return: A list of tuples as a path from the given start to the given end in the given maze
        '''
        start = (360-int(start[1]*30),int(start[0]*30))
        end = (360-int(end[1]*30),int(end[0]*30))
        
        dist = {}
        dist_heuristic = {}
        parent = {}
        dist[start] = 0
        dist_heuristic[start] = euclid(start, end)
        parent[start] = None
        
        frontier = [start]
        
        i = 0
        while len(frontier) > 0:
            i += 1
            if i % 100000 == 0:
                print(i)
            node = frontier[0]
            frontier.remove(frontier[0])
            #if node[0] == end[0] and node[1] == end[1]:
                #break
            #draw_rect(node[0], node[1], 3, 0x303030)
            #print(node)
            neighbors = get_neighbors(map, node)
            found_goal = False
            
            for neighbor, distance in neighbors:
                full_distance = dist[node] + distance
              
                dist_with_heuristic = dist[node] + distance + euclid(neighbor, end)
                if neighbor not in dist or dist_with_heuristic < dist_heuristic[neighbor]:
                    dist[neighbor] = full_distance
                    dist_heuristic[neighbor] = dist_with_heuristic
                    parent[neighbor] = node
                    frontier.append(neighbor)
                    
        #for node in parent.keys():
            #print(node, parent[node])
            
        path = []
        node = end
        while node != None:
            path.append(node)
            node = parent[node]
            
        path.reverse()
        return path

    # Part 2.1: Load map (map.npy) from disk and visualize it
    map = np.load("map.npy")
    
    print(gps.getValues()[2],gps.getValues()[0])
    print("Map loaded")

    # Part 2.2: Compute an approximation of the “configuration space”
    map = getConfigSpace(map)
  
    for disp_x in range(360):
        for disp_y in range(360):
            map[disp_x][disp_y] += 0.005
            if disp_x < 0 or disp_x >= 360 or disp_y < 0 or disp_y >= 360:
                continue
                        
            if map[disp_x][disp_y] > 0.99:
                map[disp_x][disp_y] = 0.99
                    
                g = map[disp_x][disp_y]
                byte = int(g * 255)
                byte2 = byte * 256
                byte3 = byte2 * 256
                    
                color = byte + byte2 + byte3
                #print(hex(color))
                #if map[disp_x][disp_y] > 1:
                display.setColor(color)
                display.drawPixel(disp_x, disp_y)

    # Part 2.3 continuation: Call path_planner
    #path = path_planner(map, (4.47, 8.06), (3.933, 3.133))
    path = path_planner(map, (gps.getValues()[0], gps.getValues()[2]), (3.933, 3.133))

    # Part 2.4: Turn paths into waypoints and save on disk as path.npy and visualize it
    np.save("path.npy", path)

######################
#
# Map Initialization
#
######################

# Part 1.2: Map Initialization

# Initialize your map data structure here as a 2D floating point array
#map = None # Replace None by a numpy 2D floating point array
map = np.zeros((360, 360))
waypoints = []

if mode == 'autonomous':
    # Part 3.1: Load path from disk and visualize it
    waypoints = np.load("path.npy")
    waypoints = [(point[1] / 30, (360 - point[0])/30) for point in waypoints]


state = 0 # use this to iterate through your path




while robot.step(timestep) != -1 and mode != 'planner':

    ###################
    #
    # Mapping
    #
    ###################

    ################ v [Begin] Do not modify v ##################
    # Ground truth pose
    pose_y = gps.getValues()[2]
    pose_x = gps.getValues()[0]

    n = compass.getValues()
    rad = -((math.atan2(n[0], n[2]))-1.5708)
    pose_theta = rad

    lidar_sensor_readings = lidar.getRangeImage()
    lidar_sensor_readings = lidar_sensor_readings[83:len(lidar_sensor_readings)-83]

    for i, rho in enumerate(lidar_sensor_readings):
        alpha = lidar_offsets[i]

        if rho > LIDAR_SENSOR_MAX_RANGE:
            continue

        # The Webots coordinate system doesn't match the robot-centric axes we're used to
        rx = math.cos(alpha)*rho
        ry = -math.sin(alpha)*rho

        # Convert detection from robot coordinates into world coordinates
        wx =  math.cos(pose_theta)*rx - math.sin(pose_theta)*ry + pose_x
        wy =  -(math.sin(pose_theta)*rx + math.cos(pose_theta)*ry) + pose_y
        
    
        ################ ^ [End] Do not modify ^ ##################

        #print("Rho: %f Alpha: %f rx: %f ry: %f wx: %f wy: %f" % (rho,alpha,rx,ry,wx,wy))

        if rho < LIDAR_SENSOR_MAX_RANGE:
            
            # Part 1.3: visualize map gray values.

            # You will eventually REPLACE the following 2 lines with a more robust version of the map
            # with a grayscale drawing containing more levels than just 0 and 1.
            display.setColor(0xFFFFFF)
            disp_x = 360-int((wy + 5)*30)
            disp_y = int((wx + 5)*30)
            
            #print(disp_x, disp_y, wx, wy)
            if (disp_x > 359):
                disp_x = 359
                
            if (disp_y > 359):
                disp_y = 359
                
            map[disp_x][disp_y] += 0.01
            if map[disp_x][disp_y] > 0.99:
                map[disp_x][disp_y] = 0.99
            
            g = map[disp_x][disp_y]
            byte = int(g * 255)
            byte2 = byte * 256
            byte3 = byte2 * 256
            
            color = byte + byte2 + byte3

            
            #if map[disp_x][disp_y] > 1:
            display.setColor(color)
            display.drawPixel(disp_x, disp_y)

    # Draw the robot's current pose on the 360x360 display
    display.setColor(int(0xFF0000))
    display.drawPixel(360-int(pose_y*30),int(pose_x*30))



    ###################
    #
    # Controller
    #
    ###################
    if mode == 'manual':
        key = keyboard.getKey()
        #while(keyboard.getKey() != -1): pass
        
        if key == keyboard.LEFT :
            vL = -MAX_SPEED
            vR = MAX_SPEED
        elif key == keyboard.RIGHT:
            vL = MAX_SPEED
            vR = -MAX_SPEED
        elif key == keyboard.UP:
            vL = MAX_SPEED
            vR = MAX_SPEED
        elif key == keyboard.DOWN:
            vL = -MAX_SPEED
            vR = -MAX_SPEED
        elif key == ord(' '):
            vL = 0
            vR = 0
        elif key == ord('S'):
            # Part 1.4: Filter map and save to filesystem
            np.save("map.npy",map)
            print("Map file saved")
        elif key == ord('L'):
            # You will not use this portion in Part 1 but here's an example for loading saved a numpy array
            map = np.load("map.npy")
            for disp_x in range(360):
                for disp_y in range(360):
                    map[disp_x][disp_y] += 0.005
                    if disp_x < 0 or disp_x >= 360 or disp_y < 0 or disp_y >= 360:
                        continue
                        
                    if map[disp_x][disp_y] > 0.99:
                        map[disp_x][disp_y] = 0.99
                    
                    g = map[disp_x][disp_y]
                    byte = int(g * 255)
                    byte2 = byte * 256
                    byte3 = byte2 * 256
                    
                    color = byte + byte2 + byte3
                    #print(hex(color))
                    #if map[disp_x][disp_y] > 1:
                    display.setColor(color)
                    display.drawPixel(disp_x, disp_y)
            print("Map loaded")
        else: # slow down
            vL *= 0.2
            vR *= 0.2
    else: # not manual mode
        # Part 3.2: Feedback controller
        translation_error = euclid([pose_x,pose_y],waypoints[state])
        
        if state == len(waypoints) - 1 and translation_error < 0.2:
            mode = 'done'
            vL = 0
            vR = 0
        elif (translation_error < 0.6):
            state+=1;
            if state >= len(waypoints):
                state = len(waypoints) - 1
        
            

        # print('pose',pose_x,pose_y,pose_theta*180/3.14,waypoints[state])
        rho = 0
        alpha = -(math.atan2(waypoints[state][1]-pose_y,waypoints[state][0]-pose_x) + pose_theta)


        #STEP 2: Controller
        dX = 0
        dTheta = 0
        

        translation_coef = 1;
        rotation_coeff = 1;

        prop_left = translation_coef - (rotation_coeff*alpha*AXLE_LENGTH/2)
        prop_right = translation_coef + (rotation_coeff*alpha*AXLE_LENGTH/2)
    
        if (prop_left>prop_right):
            vL = MAX_SPEED;
            vR = prop_right/prop_left*MAX_SPEED;
        else:
            vL = prop_left/prop_right*MAX_SPEED;
            vR = MAX_SPEED;
            
        if mode == 'done':
            vL = 0
            vR = 0

        # Normalize wheelspeed
        # (Keep the wheel speeds a bit less than the actual platform MAX_SPEED to minimize jerk)


    # Odometry code. Don't change vL or vR speeds after this line.
    # We are using GPS and compass for this lab to get a better pose but this is how you'll do the odometry
    pose_x += (vL+vR)/2/MAX_SPEED*MAX_SPEED_MS*timestep/1000.0*math.cos(pose_theta)
    pose_y -= (vL+vR)/2/MAX_SPEED*MAX_SPEED_MS*timestep/1000.0*math.sin(pose_theta)
    pose_theta += (vR-vL)/AXLE_LENGTH/MAX_SPEED*MAX_SPEED_MS*timestep/1000.0

        #print("X: %f Z: %f Theta: %f" % (pose_x, pose_y, pose_theta))

        # Actuator commands
    robot_parts[MOTOR_LEFT].setVelocity(vL)
    robot_parts[MOTOR_RIGHT].setVelocity(vR)