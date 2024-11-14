from jetbot_mini.msg import State_msg
import os
import sys 
import rospy
from .Topic import Topic

current_dir = '/'.join( os.path.abspath(sys.argv[0]).split('/')[:-1] )
os.chdir(current_dir)


class State(Topic):
    def __init__(self, allow_anonymous=False, queue_size=10):
        super(State, self).__init__(State_msg, allow_anonymous, queue_size)
        self.name = "state"

    def set_name(self, name):
        self.name = name

