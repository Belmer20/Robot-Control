#!/usr/bin/env python
import numpy as np
import rospy
from Controller import RobotController
from jetbot_mini.msg import Position_msg, State_msg
from Topics.Position import Position
from Topics.State import State
from threading import Thread
from utils import get_robot_conf

class P_robot(RobotController):

    def __init__(self, full_speed, speed_factor, wheel_base, node_name):
        rospy.init_node(node_name, anonymous=False)
        self.position_topic_prefix = "position_"
        self.state_topic_prefix = "state_"
        super(P_robot, self).__init__(full_speed, speed_factor, wheel_base, node_name)

        robot_conf = get_robot_conf()
        if not (robot_conf["index"] == 0):
            rospy.logerr("NOT A PRIMARY ROBOT NODE")
            return
        number_others = robot_conf["number_others"]

        self.index = robot_conf["index"]
        self.secondary_robots = [s for s in range(number_others+1) if not (s == self.index)]


        self.node_name = node_name + str(self.index)
        self.rate = rospy.Rate(1)

        self.position_topic = Position(self.position_topic_prefix + str(self.index), Position_msg)
        self.state_topic = State(self.state_topic_prefix + str(self.index), State_msg)

        self.position_topic.register_publisher(self.index, self.node_name)

        self.others_position_topics = ["position_" + str(s) for s in self.secondary_robots]
        self.others_state_topics = ["state_" + str(s) for s in self.secondary_robots]

        self.secondary_positions = []
        self.secondary_states = {str(s): False for s in self.secondary_robots}
        
        
        self.messages_to_subscribe = [
            {"position": Position_msg, "callback": self.update_secondary_position}, 
            {"state": State_msg, "callback": self.update_secondary_state}]
        
        self.threads = self.secondary_threads()


    def fetch_command(self):
        # ! read from the commands topic
        pass

    def check_status(self):
        #! check if there is no obstacle in the way
        pass

    def dispatch(self):
        # ! send commands to other robots
        pass

    def update_secondary_position(self, data):
        print(data)

    def update_secondary_state(self, message):
        index = message.index
        state = message.ready
        self.secondary_states[str(index)] = state




    def listener(self, message, index):
        keys = message.keys()
        keys.remove("callback")
        message_name = keys[0]
        prefix = message_name + "_"
        message_type = message.get(message_name)
        callback = message.get("callback")


        for ind in index:            
            topic_name = prefix + str(ind)
            rospy.Subscriber(topic_name, message_type, callback)  # Fixed line
        rospy.spin()


    def secondary_threads(self):
        threads = []
        for message in self.messages_to_subscribe:
            thread = Thread(target=self.listener, args=(message, self.secondary_robots))
            thread.daemon = True
            thread.start()
            threads.append(thread)

        return threads


    

def main_thread_function():
    pass

def main():
    
    node_name = "R_0"

    node = P_robot(0.5, 0.5, 0.13, node_name)
    rate = node.rate

    position_msg = Position_msg()

    print(node.others_position_topics)

    while not rospy.is_shutdown():
        position_msg.x = float("%.2f" % (np.random.random() * np.random.randint(100)))
        position_msg.y = float("%.2f" % (np.random.random() * np.random.randint(100)))
        node.position_topic.publish(node.node_name, position_msg)
        print(node.secondary_states)
        rate.sleep()
    
    
if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
