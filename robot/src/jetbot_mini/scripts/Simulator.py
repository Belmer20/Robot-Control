#!/usr/bin/env python

import matplotlib.pyplot as plt
import rospy
# from geometry_msgs.msg import Point  # Assuming your message type is Point
import numpy as np
import time
from threading import Thread
from jetbot_mini.msg import trajectory_msg, Time_step_msg, Obstacle_msg
import inspect
from utils import get_robot_conf

class Simulator:
    def __init__(self):

        robot_conf = get_robot_conf()

        self.robot_speed_factor = robot_conf["speed_factor"]
        self.robot_full_speed = robot_conf["full_speed"]
        
        self.update_freq = 30  # Number of iterations to update the plot
        self.trajectory_plot = self.init_plot()
        self.trajectory = {"x_t": lambda t: t, "y_t": lambda t: t}
        self.node = rospy.init_node('Simulator')
        self.delta_time = 1e-3
        self.t_f = 0
        self.step = 0
        self.x_t = []
        self.y_t = []
        self.time_space = np.linspace(0, 0, 0)
        self.plot = False
        self.restart = False
        self.timer_active = False
        self.pen_down = True
        self.plot_next = True

        # Start the subscriber thread before creating the publisher
        self.update_traj = Thread(target=self.update_traj_tread, args=())
        # self.update_traj.start()



        # time.sleep(3)
        self.obstacle = False
        # test_pub = Thread(target=self.test_pub, args=())
        # test_pub.start()
        # time.sleep(2)
        # # Create the publisher and publish the message
        self.step_pub = rospy.Publisher("time_step", Time_step_msg, queue_size=10)
        time.sleep(0.01)
        self.timer_active = True
        rospy.Timer(rospy.Duration(1), self.init_trajectory, oneshot=True)
        # rospy.Timer(rospy.Duration(5), self.test_pub, oneshot=True)
        self.timer = rospy.Timer(rospy.Duration(self.delta_time), self.update_plot)
        rospy.Subscriber('trajectory', trajectory_msg, self.updater)
        rospy.Subscriber('obstacle_Camera_0', Obstacle_msg, self.update_obstacle_state)


        rospy.Timer(rospy.Duration(2), lambda _: rospy.spin(), oneshot=True)
        
        # self.updater_listen = rospy.Timer(rospy.Duration(self.delta_time), self.update_traj_tread)
            
    def update_obstacle_state(self, msg):
        self.obstacle = msg.obstacle
        # rospy.logwarn("obstacle: {}".format(self.obstacle))
    
    def test_pub(self,_ ):
        start = [[0, 0], [0, 0], [1, 2], [2, 0] , [1.5, 1], [0.5, 1] ]
        dest =  [[0, 0], [1, 2], [2, 0], [1.5, 1], [0.5, 1], [0, 0] ]
        for s, d in zip(start, dest):
            publisher = rospy.Publisher("trajectory", trajectory_msg, queue_size=10)
            msg = trajectory_msg()
            msg.a1, msg.a2, msg.b1, msg.b2 = s[0], s[1], d[0], d[1]
            msg.t_f = 2
            msg.straight_path = True
            msg.right = True
            publisher.publish(msg)
            time.sleep(2)

    def clean_plot(self):

        rospy.logwarn("cleaning plot")
        self.x_t= []
        self.y_t = []
        self.trajectory_plot.set_data(self.x_t, self.y_t)
        self.step = 0
        self.time_space = np.array([])
        plt.draw()
        
    def updater(self, msg):
        self.plot = False
        self.timer_active = False

        if msg.new:
            self.clean_plot()
            return
            



        start_point = [msg.a1, msg.a2]
        dest_point = [msg.b1, msg.b2]
        t_f = msg.t_f

        rospy.loginfo("\n############################################\nupdating trajectory\nnew: {}\ndestination: {}\n############################################\n".format(msg.new, dest_point))
        self.t_f = t_f
        self.trajectory = self.straight_trajectory_equation(start_point, dest_point, t_f)\
              if msg.straight_path\
              else self.circular_trajectory_equations(start_point, dest_point, t_f, msg.radius, clockwise = msg.right)
        
        self.time_space = np.linspace(0, t_f, int(t_f / (2 * self.delta_time)) + 1)
        self.pen_down = msg.pen_down

        self.step = 0
        time_step_msg = Time_step_msg()

        time_step_msg.time_step = self.step
        self.step_pub.publish(time_step_msg)

        self.plot = True
        self.timer_active = True

    def update_traj_tread(self, _):
        rospy.Subscriber('trajectory', trajectory_msg, self.updater)

    def circular_trajectory_equations(self, current_point, dest_point, t_f, radius, clockwise):
        curr_x, curr_y = current_point[0], current_point[1]
        dest_x, dest_y = dest_point[0], dest_point[1]
        center_point = ( (dest_x + curr_x) / 2 ,  (dest_y + curr_y) / 2)
        
        angular_velocity = ((self.robot_full_speed * self.robot_speed_factor) / radius) * (-1 if clockwise else 1)
        initial_phase = np.arctan2(curr_y - center_point[1], curr_x - center_point[0])

        x_t = lambda t: center_point[0] + radius *  (np.cos((angular_velocity * t + initial_phase)))
        y_t = lambda t: center_point[1] + radius *  (np.sin((angular_velocity * t + initial_phase)))

        return {"x_t": x_t, "y_t": y_t}

    def straight_trajectory_equation(self, current_point, dest_point, t_f):
        curr_x, curr_y = current_point[0], current_point[1]
        dest_x, dest_y = dest_point[0], dest_point[1]

        x_t = lambda t: ((dest_x - curr_x) / t_f) * t + curr_x
        y_t = lambda t: ((dest_y - curr_y) / t_f) * t + curr_y

        return {"x_t": x_t, "y_t": y_t}

    def init_plot(self):
        # plt.ion()
        plt.figure(figsize=(9, 9))

        plt.ylim(-5, 5)  # Adjust the limits as needed
        plt.xlim(-5, 5)
        plt.xticks(np.arange(-5, 5, 0.5))
        plt.yticks(np.arange(-5, 5, 0.5))
        plt.grid(visible=True)
        trajectory_plot, = plt.plot([], [])
        return trajectory_plot

    def init_trajectory(self, _):
        rospy.loginfo("initializing trajectory")
        # curr_x, curr_y = start_point[0], start_point[1]
        msg = trajectory_msg()
        msg.a1, msg.a2, msg.b1, msg.b2 = 0, 0, 0, 0
        msg.t_f = 1e-3
        msg.straight_path = True
        msg.right = True
        msg.new = True
        publisher = rospy.Publisher("trajectory", trajectory_msg, queue_size=10)
        publisher.publish(msg)

    def update_plot(self, _):

        if not self.timer_active :
            return
        

        if self.plot and self.step < len(self.time_space) and not self.obstacle:
            rospy.loginfo("plotting, obstacle {}".format(self.obstacle))
            x_t = self.trajectory["x_t"](self.time_space[self.step])
            y_t = self.trajectory["y_t"](self.time_space[self.step])
            self.x_t.append(x_t)
            self.y_t.append(y_t)

            if ((self.step + 1) % self.update_freq == 0) and\
                  self.step < len(self.time_space) and\
                  self.pen_down:
                
                self.trajectory_plot.set_data(self.x_t, self.y_t)
                plt.draw()

            self.step += 1

            time_step_msg = Time_step_msg()

            time_step_msg.time_step = self.step
            self.step_pub.publish(time_step_msg)

        else:
            rospy.logerr("cannot move, obstacle {}\nstep: {}".format(self.obstacle, self.step))
    # def plot_trajectory(self):
    #     self.timer_active = True
    #     self.timer = rospy.Timer(rospy.Duration(self.t_f / len(self.time_space)), self.update_plot)


    # def do_plot(self, start_point, dest_point, t_f):
    #     self.init_trajectory(start_point, dest_point, t_f)
    #     self.plot_trajectory()
    #     while not self.plot:
    #         pass

    #     self.timer.shutdown()

if __name__ == '__main__':
    simulator = Simulator()
    
    # simulator.plot_trajectory()

    # simulator.timer.shutdown()    

    plt.show(block=True)

