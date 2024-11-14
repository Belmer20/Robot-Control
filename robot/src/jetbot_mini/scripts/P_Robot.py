#!/usr/bin/env python
from Controller import RobotController
from jetbot_mini.msg import Position_msg, State_msg, Commands_msg, Command_classes_msg, trajectory_msg
from Topics.Position import Position
from Topics.State import State
from Topics.Commands import Commands
from threading import Thread
from utils import get_robot_conf
import os, sys, rospy, numpy as np

# Get the current directory of the script
current_dir = os.path.dirname(os.path.abspath(__file__))

# Add the directory containing Topic.py to the Python path
topic_dir = os.path.join(current_dir, "Topics")
sys.path.append(topic_dir)


def listener(message, index):
    keys = list(message.keys())
    keys.remove("callback")
    message_name = keys[0]
    prefix = message_name + "_"
    message_type = message.get(message_name)
    callback = message.get("callback")
    for ind in index:
        topic_name = prefix + str(ind)
        rospy.Subscriber(topic_name, message_type, callback)  # Fixed line
    # rospy.spin()


class P_robot(RobotController):

    def __init__(self, full_speed, speed_factor, wheel_base, node_name, simulate= False):

        robot_conf = get_robot_conf()

        if not (robot_conf["index"] == 0):
            rospy.logerr("NOT A PRIMARY ROBOT NODE")
            raise Exception("this is a Primary node, check the conf.json file and make sure that the index is 0")
        
        conf_robot_name = node_name if robot_conf['name'] is None else robot_conf['name'] 
        self.index = robot_conf["index"]
        number_others = robot_conf["number_others"]
        self.master_name = robot_conf["master_name"]
        self.node_name = conf_robot_name + str(self.index) 

        super(P_robot, self).__init__(full_speed, speed_factor, wheel_base, self.node_name)
        self.init()
        self.rate = rospy.Rate(1)

        self.secondary_robots = [s for s in range(number_others+1) if not (s == self.index)]




        # ? Setting up Topics
        # ?##################################################################################################################
        self.position_topic_prefix = "position_"
        self.state_topic_prefix = "state_"
        self.commands_topic_prefix = "commands_"

        self.position_topic = Position()
        self.position_topic.set_name("master_position")
        self.position_topic.register_node(self.node_name)

        self.state_topic = State()
        self.state_topic.set_name("master_state")
        self.state_topic.register_node(self.node_name)

        self.others_position_topics = [ self.position_topic_prefix  + str(s) for s in self.secondary_robots]
        self.others_state_topics = [ self.state_topic_prefix  + str(s) for s in self.secondary_robots]
        self.others_cmds_topics_names = [ self.commands_topic_prefix + str(s) for s in self.secondary_robots]
        
        # TODO: updating secondary positions
        self.secondary_positions = [] 
        self.secondary_states = {str(s): False for s in self.secondary_robots}
        
        self.messages_to_subscribe = [
            {"position": Position_msg, "callback": self.update_secondary_position}, 
            {"state": State_msg, "callback": self.update_secondary_state}]
        self.threads = self.secondary_threads()

        self.commands_topic = Commands()

        self.current_command_sub = None
        self.current_command_classes_sub = None

        for s in self.secondary_robots:
            self.commands_topic.register_publisher(str(s))
        # ?##################################################################################################################



        # if not(len(new_subs) == 0) :
        #     self.position_subscribes = self.subscribe_pos(new_subs, node_name)
        # else:
        #     self.position_subscribes = []

        self.fetch_command()

    




    def init(self):
        rospy.loginfo("starting node: {}".format(self.node_name))
        rospy.init_node(self.node_name, anonymous=False)

    def fetch_command(self):

        # ! read from the commands topic (from the User)

        self.update_master_state(True)

        rospy.loginfo("waiting for commands")
        #
        self.current_command_sub = rospy.Subscriber("master_commands", Commands_msg, self.get_classes_commands)

    def get_classes_commands(self, command):
        # rospy.loginfo(f"received command \n{command}\n")
        self.current_command = command
        rospy.loginfo("Command received, waiting for its classes\ncommand: {}".format(command))
        self.current_command_sub.unregister()
        self.current_command_classes_sub = rospy.Subscriber("master_commands_classes", Command_classes_msg, self.process_command)


    def process_command(self, command_classes):

        self.update_master_state(False)
        self.current_command_classes_sub.unregister()
        rospy.loginfo("Command_classed received, executing\ncommand_classes: {}".format(command_classes))
        self.exec_command(self.current_command, command_classes)
        self.fetch_command()


    def update_master_state(self, state):
        state_msg = State_msg()
        state_msg.ready = state
        self.state_topic.publish(self.node_name, state_msg)
        print("################################\nState updated to {}\n################################\n".format(state_msg.ready))

    """
        The following are to work with multiple robots: master slave
    """
    # TODO
    def dispatch(self, commands):
        # ! send commands to other robots
        # ! commands : [{"robot_index": 1, "command": Commands_msg}]
        for command in commands:
            # print("#######################################################\n#######################################################\n
            # dispatching {} to robot index {}.\n#######################################################\n###
            # ###################################################\n
            # ".format(command.get("command"), command.get("robot_index")))
            rospy.Publisher(self.commands_topic_prefix + str(command.get("robot_index")), Commands_msg, queue_size=10).\
                publish(command.get("command"))

        pass

    # TODO
    def update_secondary_position(self, data):

        # TODO: update the position of other robots

        print(data)

    def update_secondary_state(self, message):
        index = message.index
        state = message.ready
        self.secondary_states[str(index)] = state
        # rospy.loginfo("updated state at index {}, new state:\n##################################################\n{}\n
        # ##################################################\n".format(index, self.secondary_states))

    """
        these are the threads responsible for listening to states of secondary robots
    """
    def secondary_threads(self):
        threads = []
        for message in self.messages_to_subscribe:
            thread = Thread(target=listener, args=(message, self.secondary_robots))
            thread.daemon = True
            thread.start()
            threads.append(thread)
        return threads


def main():
    
    node_name = "R_0"
    node = P_robot(0.5, 0.5, 0.13, node_name)
    # node.draw("")
    rospy.spin()
    # rate = node.rate

    # position_msg = Position_msg()

    # print(node.others_cmds_topics_names)

    # state = State_msg()
    # state.ready = bool(np.random.randint(2))
    # node.update_master_state(state)

    # node.fetch_command()
    
    
if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
