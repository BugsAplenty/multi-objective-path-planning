import matplotlib.pyplot as plt
import numpy as np
import copy

class Room(object):
    
    def map_range_x(self, start, stop, number, y):
        return [[start + (stop - start) * i / number, y] for i in range(number + 1)]

    def map_range_y(self, start, stop, number, x):
        return [[x, start + (stop - start) * i / number] for i in range(number + 1)]

    def map_obstacle(self, left, right, points):
        l_x, l_y = left
        r_x, r_y = right
        res = self.map_range_x(l_x, r_x, points, l_y)
        return res
    
    def map_square(self, top_left, bottom_right, points):
        tl_x, tl_y = top_left
        br_x, br_y = bottom_right
        res  = self.map_range_y(tl_y, br_y, points, tl_x)
        res += self.map_range_y(tl_y, br_y, points, br_x)
        res += self.map_range_x(tl_x, br_x, points, tl_y)
        res += self.map_range_x(tl_x, br_x, points, br_y)
        return res
    
    def make_room(self):
        walls = self.map_square((0.0,0.0), (10.0,10.0), 100)
        left_obstacle = self.map_obstacle((0.0,6.0), (6.0,6.0), 100)
        right_obstacle = self.map_obstacle((4.0,4.0), (10.0,4.0), 100)
        return walls + left_obstacle + right_obstacle

class RoomView(object):
    
    def __init__(self, room_points, angle_sweep, sweep_number):
        self.room_points = room_points
        self.angle_sweep = angle_sweep
        self.sweep_number = sweep_number
        
    def angle_range(self, center):
        delta = self.angle_sweep
        n = self.sweep_number
        return [(center - delta) + delta * i / n for i in range(2 * n + 1)]

    def angle_and_distance(self, robot_location, point_location):
        rx, ry = robot_location
        px, py = point_location
        dx = px - rx
        dy = py - ry
        distance = np.sqrt(np.sum(np.square(dx), np.square(dy)))
        angle = np.arctan2(dy, dx)
        return (angle, distance)

    def closest_point_in_swath(self, min_angle, max_angle, robot_location):
        pts = []
        for p in self.room_points:
            angle, distance = self.angle_and_distance(robot_location, p)
            if angle > min_angle and angle < max_angle:
                pts.append(distance)
        pts = sorted(pts)
        if len(pts) > 0:
            return pts[0]
        return None

    def lidar_observations_polar(self, robot_location, robot_angle):
        pts = []
        angles = self.angle_range(robot_angle)
        delta = self.angle_sweep / self.sweep_number
        for a in angles:
            min_a = a - delta / 2.0
            max_a = a + delta / 2.0
            distance = self.closest_point_in_swath(min_a, max_a, robot_location)
            if distance is not None:
                pts.append((a, distance))
        return pts
    
# Set the plot size        
plt.figure(figsize=(8,8))    
    
# Build the room map points
r = Room()
room = r.make_room()

# Unzip the x-y coordinates
x, y = zip(*room)

# Plot the points
plt.scatter(x, y)  

# Plot where our robot is 
plt.scatter(5.0, 1.0, color='green')

# Plot where your goal is
plt.scatter(5.0, 9.0, color='red')

#  Set up a room view with a pi/4 half-angle aperture and 8 angle slices on each side of center
rv = RoomView(room, np.pi/4, 8)

# Get the lidar points for a particle
points = rv.lidar_points((10.0,5.0), 0.0)

plt.show()