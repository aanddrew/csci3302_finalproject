"""hider."""
# canabalized from lab5_controller
from controller import Robot, Motor, Camera, RangeFinder, Lidar, Keyboard
import math
import numpy as np
from matplotlib import pyplot as plt
from scipy.signal import convolve2d # Uncomment if you want to use something else for finding the configuration space
import RRT_class

MAX_SPEED = .25*7.0  # [rad/s]
MAX_SPEED_MS = 0.633 # [m/s]
AXLE_LENGTH = 0.50 # m
WHEEL_RADIUS = 0.1 # m
MOTOR_LEFT = 0
MOTOR_RIGHT = 1
N_PARTS = 2

LIDAR_ANGLE_BINS = 667
LIDAR_SENSOR_MAX_RANGE = 2.75 # Meters
LIDAR_ANGLE_RANGE = math.radians(240)

DISPLAY_LENGTH = 360 #number of pixels on a side of display
WORLD_XLIMS = (-10,10)
WORLD_YLIMS = (-10,10)

mode = "explore"
##### vvv [Begin] Do Not Modify vvv #####

################FUNCTIONS########################
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

def clear_display():
    display.setColor(0x000000)
    display.fillRectangle(0, 0, DISPLAY_LENGTH, DISPLAY_LENGTH)

def draw_tree(nodes):


    for node in nodes:
        if node.parent is None: continue
        # print(node.parent.point)

        draw_rect(node.point[0], node.point[1], 3, 0xff0000)
        display.setColor(0xff00ff)
        # print('draw tree',node.point[0], node.point[1], node.parent.point[0], node.parent.point[1])
        display.drawLine(int(node.point[0]), int(node.point[1]), int(node.parent.point[0]), int(node.parent.point[1]))

    # Draw a different color for the path
    display.setColor(0x00ff10)
    cur_node = nodes[len(nodes)-1]
    while cur_node.parent is not None:
        display.drawLine(int(cur_node.point[0]), int(cur_node.point[1]), int(cur_node.parent.point[0]), int(cur_node.parent.point[1]))
        cur_node = cur_node.parent

    #start node
    draw_rect(nodes[0].point[0], nodes[0].point[1], 5, 0x00ff00)
    #end node
    draw_rect(nodes[len(nodes)-1].point[0], nodes[len(nodes)-1].point[1], 5, 0xffff00)

def world_to_pixel(point):
    x,y = point
    row = int(DISPLAY_LENGTH/(WORLD_XLIMS[1]-WORLD_XLIMS[0])*(x-WORLD_XLIMS[0]))
    col = int(-DISPLAY_LENGTH/(WORLD_YLIMS[1]-WORLD_YLIMS[0])*(y-WORLD_YLIMS[0])+DISPLAY_LENGTH)
    return (col,row)

def pixel_to_world(points):
    coords = []
    for i,point in enumerate(points):
        x = point[0]
        y = point[1]
        world_x = -x*(WORLD_XLIMS[0]-WORLD_XLIMS[1])/DISPLAY_LENGTH+WORLD_XLIMS[0]
        world_y = (y-DISPLAY_LENGTH)*(WORLD_YLIMS[0]-WORLD_YLIMS[1])/DISPLAY_LENGTH+WORLD_YLIMS[0]
        coords.append(np.array([-world_y,-world_x]))
    return coords

def draw_rect(x, y, radius, color):
    x = int(x)
    y = int(y)
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

def euclid(x, y):
    return np.sqrt((x[0] - y[0])**2 + (x[1] - y[1]) **2)


####################END FUNCTIONS##################

# create the Robot instance.
robot = Robot()
print('robot',robot)
# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())
DELTA_TIME = 1/timestep;


# The Tiago robot has multiple motors, each identified by their names below
part_names = ("wheel_left_joint", "wheel_right_joint")

# All motors except the wheels are controlled by position control. The wheels
# are controlled by a velocity controller. We therefore set their position to infinite.
target_pos = ('inf', 'inf')
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
goal_point = world_to_pixel((7,-2))
#initialize RRT class
map = np.zeros([360,360])
RRT = RRT_class.RRT(config_radius=10,k=200,q=50,map_size=360,min_goal_dist=2)

#do one iteration of run_RRT
robot.step(timestep)
pose_y = gps.getValues()[2]
pose_x = gps.getValues()[0]
nodes = RRT.run_RRT(world_to_pixel((pose_x,pose_y)),goal_point,map)
pixel_waypoints = RRT.get_waypoints(nodes)
waypoints = pixel_to_world(pixel_waypoints)
draw_tree(nodes)

# cuurent waypoint
display.drawOval(int(pixel_waypoints[1][0]), int(pixel_waypoints[1][1]), 10, 10)

print('here')
state = 1;
goalPoint = world_to_pixel((7,-2)) # make this random?
while robot.step(timestep) != -1:
    pose_y = gps.getValues()[2]
    pose_x = gps.getValues()[0]

    n = compass.getValues()
    rad = -((math.atan2(n[0], n[2]))-1.5708)
    pose_theta = rad

    lidar_sensor_readings = lidar.getRangeImage()
    lidar_sensor_readings = lidar_sensor_readings[83:len(lidar_sensor_readings)-83]

    # lidar stuff
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


        if rho < LIDAR_SENSOR_MAX_RANGE:
            # Part 1.3: visualize map gray values.

            # You will eventually REPLACE the following 2 lines with a more robust version of the map
            # with a grayscale drawing containing more levels than just 0 and 1.
            display.setColor(0xFFFFFF)
            temp = world_to_pixel((wx,wy))
            disp_x = temp[0]
            disp_y = temp[1]
            if (disp_x > 359):
                disp_x = 359

            if (disp_y > 359):
                disp_y = 359

            map[disp_x][disp_y] += 0.005
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

    display.setColor(int(0xFF0000))
    temp = world_to_pixel((pose_x,pose_y))

    x = temp[0]
    y = temp[1]

    display.drawPixel(x,y)

###################### RRT ######################################

    # print('nodes',nodes)
    # print('waypoints',RRT.get_waypoints(nodes))
    # draw_tree(nodes)

    # print('world waypoints',waypoints)
    # print('poses', pose_x, pose_y, waypoints[state])
    translation_error = euclid([pose_x, pose_y], waypoints[state])
    if state == len(waypoints) - 1 and translation_error < 0.5:
        mode = 'done'
        vL = 0
        vR = 0
    # Is the RRT needs to be rebuilt
    elif (translation_error < 0.5) or RRT.check_collisions_along_path(nodes):
        # state+=1;
        # if state >= len(waypoints):
        #     state = len(waypoints) - 1
        pixel_coords = world_to_pixel((pose_x,pose_y))
        print('pose',(pose_x,pose_y))
        # print('pixel_coords',pixel_coords)
        nodes = RRT.run_RRT(pixel_coords,goalPoint,map)
        pixel_waypoints = RRT.get_waypoints(nodes)

        waypoints = pixel_to_world(pixel_waypoints)
        print('#############################################################################')
        print('pixel ways', pixel_waypoints)
        print('State = ',state)
        print('goal wayppint', waypoints[state])
        clear_display();
        draw_tree(nodes)
        displayMap(map)
        #raw current node that the robot is travelling to
        display.drawOval(int(pixel_waypoints[1][0]), int(pixel_waypoints[1][1]), 10, 10)
        continue


    # print('pose',pose_x,pose_y,pose_theta*180/3.14,waypoints[state])
    rho = 0
    alpha = -(math.atan2(waypoints[state][1]-pose_y,waypoints[state][0]-pose_x) + pose_theta)

    if alpha > 3.14: alpha -= 6.28
    if alpha < -3.14: alpha += 6.28


######################CONTROLLER################################
    dX = 0
    dTheta = 0


    translation_coef = 1;
    rotation_coeff = 10;

    prop_left = translation_coef*translation_error - (rotation_coeff*alpha*AXLE_LENGTH/2)
    prop_right = translation_coef*translation_error + (rotation_coeff*alpha*AXLE_LENGTH/2)
    print('props',prop_left,prop_right)
    if (prop_left>prop_right):
        vL = MAX_SPEED;
        vR = prop_right/prop_left*MAX_SPEED;
    else:
        vL = prop_left/prop_right*MAX_SPEED;
        vR = MAX_SPEED;

    if alpha>.5:
        vL = -MAX_SPEED/2
        vR = MAX_SPEED/2;
    elif alpha<-.5:
        vL = MAX_SPEED/2
        vR = -MAX_SPEED/2;
    # else:
    #     vL = MAX_SPEED/2
    #     vR = MAX_SPEED/2;

    if mode == 'done':
        vL = 0
        vR = 0
    # print('velocities',vL,vR,'props',prop_left,prop_right)

    #odometry
    # pose_x += (vL+vR)/2/MAX_SPEED*MAX_SPEED_MS*timestep/1000.0*math.cos(pose_theta)
    # pose_y -= (vL+vR)/2/MAX_SPEED*MAX_SPEED_MS*timestep/1000.0*math.sin(pose_theta)
    # pose_theta += (vR-vL)/AXLE_LENGTH/MAX_SPEED*MAX_SPEED_MS*timestep/1000.0

    x_dot_robot = WHEEL_RADIUS/2*(vL+vR)
    w_dot_robot = WHEEL_RADIUS/AXLE_LENGTH*(vR-vL)

    pose_x_dot     = math.cos(pose_theta)*x_dot_robot
    pose_y_dot     = math.sin(pose_theta)*x_dot_robot
    pose_theta_dot = w_dot_robot

    pose_x         += pose_x_dot*DELTA_TIME
    pose_y         += pose_y_dot*DELTA_TIME
    pose_theta     += pose_theta_dot*DELTA_TIME

    if pose_theta > 3.14: pose_theta -= 6.28
    if pose_theta < -3.14: pose_theta += 6.28

    print('translation_error',translation_error, ' bearing error', alpha,' vels ', vL,vR)
    #set wheel vels
    robot_parts[MOTOR_LEFT].setVelocity(vL)
    robot_parts[MOTOR_RIGHT].setVelocity(vR)
# Functions
