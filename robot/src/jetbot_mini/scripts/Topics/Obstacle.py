from jetbot_mini.msg import Obstacle_msg
import os
import sys 
from .Topic import Topic

current_dir = '/'.join( os.path.abspath(sys.argv[0]).split('/')[:-1] )
os.chdir(current_dir)

class Obstacle(Topic):
    def __init__(self, allow_anonymous=False, queue_size=10):
        self.name = "obstacle"
        super(Obstacle, self).__init__(Obstacle_msg, allow_anonymous, queue_size)

    def set_name(self, name):
        self.name = name