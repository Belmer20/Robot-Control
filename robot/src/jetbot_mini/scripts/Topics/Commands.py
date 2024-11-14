from jetbot_mini.msg import Commands_msg
import os
import sys 
from .Topic import Topic

current_dir = '/'.join( os.path.abspath(sys.argv[0]).split('/')[:-1] )
os.chdir(current_dir)



class Commands(Topic):
    def __init__(self, allow_anonymous=False, queue_size=10):
        super(Commands, self).__init__(Commands_msg, allow_anonymous, queue_size)
        self.name='commands'