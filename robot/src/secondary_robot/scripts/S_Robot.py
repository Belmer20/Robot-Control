#!/usr/bin/env python


from Controller import RobotController
from jetbot_mini.msg import Position_msg, State_msg, Commands_msg
from Topics.Position import Position
from Topics.State import State
from Topics.Commands import Commands
from Commands_queue import Commands_queue
import rospy
from utils import get_robot_conf
import numpy as np
from threading import Thread
import time

class S_robot(RobotController):
    def __init__(self, full_speed, speed_factor, wheel_base, node_name):
        robot_conf = get_robot_conf()
        if  (robot_conf["index"] == 0):
            rospy.logerr("SHOULD NOT BE A PRIMARY ROBOT NODE")
            raise Exception("this is a secondary node, check the conf.json file and make sure that the index is not 0")
        number_others = robot_conf["number_others"]

        self.index = robot_conf["index"]
        self.secondary_robots = [s for s in range(number_others+1) if not (s == self.index)]
        self.master_name = robot_conf["master_name"]

        
        conf_robot_name = node_name if robot_conf['name'] is None else robot_conf['name'] 
        self.node_name = conf_robot_name + str(self.index) 


        self.init()
        self.rate = rospy.Rate(0.5)
        self.position_topic_prefix = "position_"
        self.state_topic_prefix = "state_"
        self.commands_topic_prefix = "commands_"
        self.position_topic_name = self.position_topic_prefix + str(self.index)
        self.state_topic_name = self.state_topic_prefix + str(self.index)
        super(S_robot, self).__init__(full_speed, speed_factor, wheel_base, self.node_name)

        self.position_topic = Position(self.position_topic_name, Position_msg)
        self.position_topic.register_publisher( self.index, self.node_name)

        self.state_topic = State(self.state_topic_name, State_msg)
        self.state_topic.register_publisher(self.index, self.node_name)

        self.commands_topics = [self.commands_topic_prefix+str(self.index)]
        self.master_cmd_thread = self.master_commands_thread()

        self.commands_pipeline = Commands_queue(10, Commands_msg)

        # self.commands_topic = Commands()
        # self.commands_topic.register_publisher(0, self.master_name)
        self.state = True
        self.action()

        

    def init(self):
        rospy.loginfo("starting secondary node: {}".format(self.node_name))
        rospy.init_node(self.node_name, anonymous=False)
        rospy.loginfo("started")


    def update_state(self, state):
        # publisher = rospy.Publisher(self.state_topic_name, State_msg, queue_size=10)

        msg = State_msg()
        msg.index = self.index
        msg.ready = state

        self.state_topic.publish(self.node_name, msg)
        # rospy.loginfo("updated state of index {} to {}".format(self.index, state))

    def update_commands(self, command):
        print("pushing {}".format(command))
        self.commands_pipeline.push(command)

    def commands_listener(self):
        rospy.Subscriber(self.commands_topics[0], Commands_msg, self.update_commands)
        rospy.spin()

    def master_commands_thread(self):
        thread = Thread(target=self.commands_listener, args=())
        thread.start()
        return thread
            

    # def get_current_message(topic):
    #     try:
    #         message = rospy.wait_for_message(topic, Position_msg, timeout=0.1)  
    #         return message.data 
    #     except rospy.ROSException:
    #         return None  

    def action(self):
        while not self.state:
            pass

        command = self.commands_pipeline.get()
        if command:
            pass

        



    # def sub_to_others_positions(self, number_others):
    #     secondary_robots = [{"index": s, "callback": self.update_others_position} for s in range(number_others) if not (s == self.index)]
    #     self.subscribe_pos(secondary_robots)
    #     self.others_positions = [{"index": s, "x": s, "y": 0} for s in range(number_others) if not (s == self.index)]
    #     return [s for s in range(number_others) if not (s == self.index)]

def main():

    node_name = "secondary_2"

    R_1 = S_robot(0.5, 0.5, 0.13, node_name)
    

    # print(R_1.position_subscribes)

 
    
    while not rospy.is_shutdown():
        state = bool(np.random.randint(2))
        R_1.update_state(state)

        # if R_1.commands_pipeline.elements_count() != 0:
        #     print(R_1.commands_pipeline)
            

        R_1.rate.sleep()

if __name__ == "__main__":
    
    
    try:
        main()
    except rospy.ROSInterruptException:
        pass
