import numpy as np
import os 
import sys
import json

current_dir = '/'.join( os.path.abspath(sys.argv[0]).split('/')[:-1] )
os.chdir(current_dir)
conf_file_path = "./conf.json"

def angle_vectors(source, destination, deg=True):
    magnitudes = np.linalg.norm(source) * np.linalg.norm(destination)
    cos = float("%.2f" % (np.dot(destination, source) / magnitudes))
    print("cos: {}".format(cos))
    angle = np.arccos(cos)
    return np.degrees(angle) if deg else angle

def directed_angle(source, destination, deg=True):
    cross = np.cross(source, destination)
    angle = angle_vectors(source, destination, deg=deg)
    print("cross between: {} and destination:  {}".format(source, destination, cross))
    if cross < 0:
        counter_clock = False
    elif cross >= 0:
        counter_clock = True
    return angle, counter_clock


def get_robot_conf():
    with open(conf_file_path, 'r') as conf_file:
        robot_conf = json.load(conf_file)["robot_node"]
        return robot_conf