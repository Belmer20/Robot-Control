import numpy as np
import math
import json
import sys
import os 

current_dir = '/'.join( os.path.abspath(sys.argv[0]).split('/')[:-1] )
os.chdir(current_dir)
conf_file_path = "./conf.json"

def angle_vectors(source, destination, deg=True):
    
    dot_product = source[0] * destination[0] + source[1] * destination[1]
    
    source_magnitude = math.sqrt(source[0]**2 + source[1]**2)
    destination_magnitude = math.sqrt(destination[0]**2 + destination[1]**2)
    
    angle = math.acos(dot_product / (source_magnitude * destination_magnitude))
    
    cross_product = source[0] * destination[1] - source[1] * destination[0]
    if cross_product < 0:
        angle = -angle

    return np.degrees(angle) if deg else angle

def directed_angle(source, destination, deg=True):
    cross = np.cross(source, destination)
    angle = angle_vectors(source, destination, deg=deg)
    # print("cross between: {} and destination:  {}".format(source, destination, cross))
    if cross < 0:
        counter_clock = False
    elif cross >= 0:
        counter_clock = True
    return angle, counter_clock


def get_robot_conf():
    with open(conf_file_path, 'r') as conf_file:
        robot_conf = json.load(conf_file)["robot_node"]
        return robot_conf
    

def check_float(n):
    if isinstance(n, bool):
        return False
    try:
        float(n)
        return True
    except:
        return False
    

class LIFO:
    def __init__(self, size, dtype):    
        self.size = size
        self.content = []
        self.index = -1
        self.dtype = dtype

    def dequeue(self):
        if self.is_empty():
            return None

        value = self.content[self.index]
        self.content[self.index] = None  # Avoid loitering
        self.index -= 1
        return value

    def enqueue(self, data):
        if not isinstance(data, self.dtype):
            return -1

        if self.is_full():
            return -2  # Stack is full

        self.index += 1
        self.content.append(data)
        return 0

    def is_empty(self):
        return self.index == -1

    def is_full(self):
        return self.index == self.size - 1


class FIFO:
    def __init__(self, size, dtype):
        self.size = size
        self.content = [None] * size
        self.front = 0
        self.rear = 0
        self.dtype = dtype

    def enqueue(self, data):
        if not isinstance(data, self.dtype):
            return -1

        if self.is_full():
            return -2  # Queue is full

        self.content[self.rear] = data
        self.rear = (self.rear + 1) % self.size

        return 0

    def dequeue(self):
        if self.is_empty():
            return None

        data = self.content[self.front]
        self.content[self.front] = None  # Avoid loitering
        self.front = (self.front + 1) % self.size

        return data

    def is_empty(self):
        return self.front == self.rear

    def is_full(self):
        next_rear = (self.rear + 1) % self.size
        return next_rear == self.front


class Queue:
    def __init__(self, size, dtype, mode='FIFO'):
        self.size = size
        self.content = [None] * size
        self.front = 0
        self.rear = 0
        self.dtype = dtype
        self.mode = mode.upper()

    def enqueue(self, data):
        if not isinstance(data, self.dtype):
            return -1

        if self.is_full():
            return -2  # Queue is full

        if self.mode == 'FIFO':
            self.content[self.rear] = data
            self.rear = (self.rear + 1) % self.size
        elif self.mode == 'LIFO':
            self.rear += 1
            self.content[self.rear] = data

        return 0

    def dequeue(self):
        if self.is_empty():
            return None

        if self.mode == 'FIFO':
            data = self.content[self.front]
            self.content[self.front] = None  # Avoid loitering
            self.front = (self.front + 1) % self.size
        elif self.mode == 'LIFO':
            data = self.content[self.rear]
            self.content[self.rear] = None  # Avoid loitering
            self.rear -= 1

        return data

    def is_empty(self):
        if self.mode == 'FIFO':
            return self.front == self.rear
        elif self.mode == 'LIFO':
            return self.rear == -1

    def is_full(self):
        if self.mode == 'FIFO':
            next_rear = (self.rear + 1) % self.size
            return next_rear == self.front
        elif self.mode == 'LIFO':
            return self.rear == self.size - 1
        
def circular_trajectory_equations(speed, current_point, dest_point, t_f, radius, clockwise):
    curr_x, curr_y = current_point[0], current_point[1]
    dest_x, dest_y = dest_point[0], dest_point[1]
    center_point = ( (dest_x + curr_x) / 2 ,  (dest_y + curr_y) / 2)
        
    angular_velocity = ((speed) / radius) * (-1 if clockwise else 1)
    initial_phase = np.arctan2(curr_y - center_point[1], curr_x - center_point[0])

    x_t = lambda t: center_point[0] + radius *  (np.cos((angular_velocity * t + initial_phase)))
    y_t = lambda t: center_point[1] + radius *  (np.sin((angular_velocity * t + initial_phase)))

    return {"x_t": x_t, "y_t": y_t}

def straight_trajectory_equation(current_point, dest_point, t_f):
    curr_x, curr_y = current_point[0], current_point[1]
    dest_x, dest_y = dest_point[0], dest_point[1]

    x_t = lambda t: ((dest_x - curr_x) / t_f) * t + curr_x
    y_t = lambda t: ((dest_y - curr_y) / t_f) * t + curr_y
    return {"x_t": x_t, "y_t": y_t}