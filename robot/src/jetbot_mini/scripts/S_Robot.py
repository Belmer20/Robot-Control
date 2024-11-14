#!/usr/bin/env python


from Controller import RobotController
from jetbot_mini.msg import Position_msg, State_msg
from Topics.Position import Position
from Topics.State import State
import rospy
from utils import get_robot_conf
import numpy as np

class S_robot(RobotController):
    def __init__(self, full_speed, speed_factor, wheel_base, node_name):
        robot_conf = get_robot_conf()
        if  (robot_conf["index"] == 0):
            rospy.logerr("SHOULD NOT BE A PRIMARY ROBOT NODE")
            raise Exception("this is a secondary node, check the conf.json file and make sure that the index is not 0")
            
        number_others = robot_conf["number_others"]

        self.index = robot_conf["index"]
        self.secondary_robots = [s for s in range(number_others+1) if not (s == self.index)]

        conf_robot_name = node_name if robot_conf['name'] is None else robot_conf['name'] 
        self.node_name = conf_robot_name + str(self.index) 

        rospy.init_node(self.node_name, anonymous=False)
        self.rate = rospy.Rate(0.5)

        self.position_topic_prefix = "position_"
        self.state_topic_prefix = "state_"
        self.position_topic_name = self.position_topic_prefix + str(self.index)
        self.state_topic_name = self.state_topic_prefix + str(self.index)
        super(S_robot, self).__init__(full_speed, speed_factor, wheel_base, self.node_name)

        self.position_topic = Position(self.position_topic_name, Position_msg)
        self.position_topic.register_publisher(self.node_name)

        self.state_topic = State(self.state_topic_name, State_msg)
        self.state_topic.register_publisher(self.node_name)


    def update_state(self, state):
        publisher = rospy.Publisher(self.state_topic_name, State_msg, queue_size=10)
        publisher.publish('')

    def commands_listener(self):
        pass
            

    def get_current_message(topic):
        try:
            message = rospy.wait_for_message(topic, Position_msg, timeout=0.1)  
            return message.data 
        except rospy.ROSException:
            return None  

    def receive(self, data):
        print(data)

    # def sub_to_others_positions(self, number_others):
    #     secondary_robots = [{"index": s, "callback": self.update_others_position} for s in range(number_others) if not (s == self.index)]
    #     self.subscribe_pos(secondary_robots)
    #     self.others_positions = [{"index": s, "x": s, "y": 0} for s in range(number_others) if not (s == self.index)]
    #     return [s for s in range(number_others) if not (s == self.index)]

def main():

    node_name = "R_1"

    R_1 = S_robot(0.5, 0.5, 0.13, node_name)
    

    # print(R_1.position_subscribes)
    
    while not rospy.is_shutdown():
        msg = State_msg()
        msg.index = R_1.index
        msg.ready = bool(np.random.randint(2))

        R_1.state_topic.publish(R_1.node_name, msg)
        R_1.rate.sleep()

if __name__ == "__main__":
    
    
    try:
        main()
    except rospy.ROSInterruptException:
        pass
