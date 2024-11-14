from jetbot_mini.msg import Position_msg
import os
import sys 
from .Topic import Topic
current_dir = '/'.join( os.path.abspath(sys.argv[0]).split('/')[:-1] )
os.chdir(current_dir)




class Position(Topic):
    def __init__(self, allow_anonymous=False, queue_size=10):
        self.name = "position"
        super(Position, self).__init__(Position_msg, allow_anonymous, queue_size)
    def set_name(self, name):
        self.name = name
