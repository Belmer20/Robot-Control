#!/usr/bin/env python

from jetbot_mini.msg import Commands_msg, Command_classes_msg, trajectory_msg, State_msg, Obstacle_msg, Time_step_msg
from utils import angle_vectors, directed_angle, check_float, straight_trajectory_equation, circular_trajectory_equations
from threading import Thread
from coords import Coords
import matplotlib.pyplot as plt
import numpy as np
import subprocess
import rospy
import time
import math


class RobotController(object):

    def __init__(self, full_speed, speed_factor, wheel_base, node_name, new_subs = []):

        self.wheel_base = wheel_base
        self.full_speed = full_speed
        self.speed_factor = speed_factor
        self.speed = self.speed_factor * self.full_speed

        self.pre_movement_buzz_time = 2
        self.post_movement_buzz_time = 2
        self.inter_steps_buzz_time = 0.5

        self.trajectory_topic_name = 'trajectory'
        self.trajectory_pub = rospy.Publisher(self.trajectory_topic_name, trajectory_msg, queue_size=10)
        time.sleep(0.1) 

        self.defaults = {"distance": 1, "turn": "right", "theta": 90, "angle_unit": "deg"}

        #! attributes related to position
        #!#########################################################

        self.coordinates = np.array([0, 0])
        self.prev_coords = np.array([0, 0])
        self.prev_step = {"vector": False, "arc": [False, 0]}
        self.angle_with_x_ax = 0

        #!#########################################################

        self.right_angle_time = 0.5
        
        self.distance_factor=1
        
        self.all_coords = Coords().all_coords
        self.all_shapes = Coords().all_shapes

        self.previous_locations = [[0], [0]]

        self.state = False

        self.action_method_map = {
            "turns": self.turn,
            "straight": self.forward,
            "shapes": self.draw
        }

        self.methods_args = {
            self.forward : {"distance": float},
            self.turn: {"theta": float, "deg": bool, "right": bool},
            self.draw: {"text": str}
        }
        self.delta_time = 1e-3
        self.obstacle = False

        self.active_trajectory = {"x_t": lambda x: x, "y_t": lambda x: x}
        self.active_timespace = np.linspace(0, 0, 0)


        
        self.obstacle_subscriber = rospy.Subscriber("obstacle_Camera_0", Obstacle_msg, self.update_obstacle_state)
        self.step_sub = rospy.Subscriber("time_step", Time_step_msg, self.update_time_step)

        self.active_dest_point = [0, 0]
        self.active_method = ''

        self.obstacle_checker_thread = Thread(target=self.obstacle_checker)
        self.obstacle_checker_thread.daemon = True
        self.obstacle_checker_thread.start()

        self.time_step = 0


    def obstacle_checker(self):
        
        changed = True
        while True:
            if self.obstacle and changed:
                rospy.logwarn("emergency stop")
                changed = False

                # current_time_step = self.time_step

                # self.coordinates[0], self.coordinates[1] = self.active_trajectory["x_t"](self.active_timespace[current_time_step]), self.active_trajectory["y_t"](self.active_timespace[current_time_step])

                # msg = trajectory_msg()
                # msg.a1, msg.a2, msg.b1, msg.b2 = self.coo
                self.stop()
                # time.sleep(1)
                # self.clean_simulation()
                # rospy.logwarn("robot stopped at:\ncoordinates{}\nat time step: {}\ngoing to {}".format(self.coordinates, self.time_step, self.active_dest_point))

                # while self.obstacle:
                #     pass

                # if self.active_method == 'forward':
                #     self.forward(0, self.active_dest_point, True)
                


                
            
            elif not self.obstacle:
                # rospy.loginfo("robot moving...")
                # TODO:
                # * Implement a logic to resume the path when the obstacle is cleared
                changed = True

            time.sleep(1e-6)


    # ! obstacle Logic
    # ########################################################################################
    def update_time_step(self, msg):
        self.time_step = msg.time_step

    def update_obstacle_state(self, msg):
        self.obstacle = msg.obstacle
    
    def update_state(self, node_name, state_topic, state):
        state_msg = State_msg()
        state_msg.ready = state
        state_topic(node_name, state_msg)
        rospy.loginfo("################################\nState updated to {}\n################################\n".format(state_msg.ready))
    # ########################################################################################
    def publish_trajectory(self, traj_msg):
        if not self.obstacle:
            self.trajectory_pub.publish(traj_msg)


    def clean_simulation(self):

        rospy.loginfo("calling to clean plot")
        traj_msg = trajectory_msg()

        traj_msg.new = True
        traj_msg.pen_down = False

        # pub = rospy.Publisher(self.trajectory_topic_name, trajectory_msg, queue_size=10)

        
        self.publish_trajectory(traj_msg=traj_msg)

    # ! Commands execution logic
    # ########################################################################################

    # ? Pipeline starts here
    def exec_command(self, command, cmd_classes):

        # checks is command and command classes objects are of the right types
        if isinstance(command, Commands_msg) and isinstance(cmd_classes, Command_classes_msg):
            # get needed actions
            action, subject, value, unit = command.action, command.subject, command.value, command.unit
            classes = cmd_classes

            rospy.logerr("command: ".format(command))


            # check if action is mapped to a method
            if self.action_method_map.get(classes.action):
                # get the method mapped to the action
                method_to_call = self.action_method_map.get(classes.action)

                # get the arguments of the method
                args = self.extract_args(method_to_call, command)

                # check if args are valid for the method
                if self.validate_call(method_to_call, args):
                    # call the method with the extracted args
                    method_to_call(**args)

                  

    # ? second function to be called
    # exctrats arguments from a given command
    def extract_args(self, method, command):

        # this is hardcoded
        # TODO: think about a way to make it dynamic
        if method == self.forward:
            # if the distance is not specified in the command,
            # it will be assumed to be the default value from the class attributes
            distance = self.defaults["distance"] if command.value == "" else command.value
            return {"distance": distance}

        elif method == self.turn:
            theta = self.defaults["theta"] if command.value == "" else command.value
            right = self.defaults["turn"] if command.subject == "" else command.subject
            deg = self.defaults["angle_unit"] if command.unit == "" else command.unit
            return {"theta": theta, "right": right == 'right', "deg": deg in ['d', 'deg', 'degree', 'degrees']}

        elif method == self.draw:
            return {"text": command.subject}

    # ? last method to call before execution
    # validates that the args are valid for the given method
    def validate_call(self, method, args):

        # get the list of required args for the given method
        requirements = self.methods_args.get(method)

        print("validatin call to {} with args {}".format(method, args))
        valid = True

        # for every requirement
        for req in requirements.keys():

            # check if the args to verify contain this requirement
            if not (req in args.keys()):
                valid = False
                break

            # get the arg from the args to verify
            arg = args.get(req)

            # in case of a float value, convert it to float
            if check_float(arg):
                arg = float(arg)

            # check if the arg is of the right data type
            if not isinstance(arg, requirements.get(req)):
                valid = False
                break

        return valid

    # ########################################################################################

    def subscribe_pos(self, new_subs):
        for sub in new_subs:
            # ! new_subs is a list with the callbacks functions for every new subscriber
            topic_name = "position_R_"+str(sub.get("index"))
            # print #("subscribing to {}".format(topic_name))
            subscriber = self.position_topic.get_subscriber(topic_name=topic_name, callback=sub.get("callback"))
            self.position_subscribes.append({"topic_name":topic_name, "subscriber": subscriber})

    def update_prev_locations(self, new_location):

        self.previous_locations[0].append(new_location[0])
        self.previous_locations[1].append(new_location[1])
        # self.plot_steps()
        return True



    # ! functions that make the robot move
    # !###############################################################

    def draw(self, text):
        # if self.obstacle:
        #     print("aborting")
        #     return
        for token in text.strip().split(" "):
            done = False
            # ! checks if it is a shape 
            
            for shapes in self.all_shapes:
                if token.lower() in shapes.keys():
                    
                    # self.clean_simulation()
                    print("drawing ##### {} ####".format(token.lower()))
                    time.sleep(2)
                    shape_coords = shapes[token.lower()]
                    initial_point = shape_coords[0]

                    # ! move to initial point, should not be plotted
                    if not (initial_point==self.coordinates).all():
                        while self.obstacle:
                            print("obstacle!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
                            continue
                        point_coordinates = np.array([initial_point['x'], initial_point['y']])
                        print("going to initial\n {}".format(point_coordinates))
                        angular_coords = self.get_angle_distance(point_coordinates)
                        self.move_to(angular_coords, dest_point=point_coordinates, simulate=False)
                        self.prev_step = {"vector": True, 'arc': [False, 0]}
                        self.update_position(point_coordinates)
                        self.update_prev_locations(point_coordinates)

                        # self.start_shape_beep()
                    self.clean_simulation()
                    for point in shape_coords[1:]:
                        while self.obstacle:
                            print("obstacle!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
                            continue
                        point_coordinates = np.array([point['x'], point['y']])
                    
                        if not (point_coordinates == self.coordinates).all():
                            arc = False
                            right = 0
                            if not point['circular']:
                                arc = False
                                    
                                angular_coords = self.get_angle_distance(point_coordinates)
                                # #print #("####################################################")
                                # #print #("from {} to {}: {}".format(self.coordinates, point_coordinates, angular_coords))
                                # #print #("####################################################")

                                #! call to move
                                self.move_to(angular_coords, dest_point=point_coordinates, simulate=True)
                                    # self.prev_step = {"vector": True, 'arc': [False, 0]}
                            else:
                                arc = True
                                    
                                right = point['right']
                                radius = self.get_radius(point_coordinates)
                                # #print #("####################################################")
                                # #print #("radius: {} to the right?: {} to {}".format(radius, right, point_coordinates))
                                # #print #("####################################################")
                                self.prev_step = {"vector": False, 'arc': [True, int(right)]}

                                #! call to move
                                self.move_circular(radius=radius, dest_point=point_coordinates,right=right, simulate = True)

                            self.update_position(point_coordinates)
                            self.update_prev_locations(point_coordinates)
                            # #print #("new coord: {}\nprev_coords: {}".format(self.coordinates, self.previous_locations))
                            # self.update_position(arc, right, point_coordinates)
                            # self.prev_coords = np.copy(self.coordinates)
                            # self.coordinates = np.array([point_coordinates[0], point_coordinates[1]])
                                

                        # self.end_step_beep()
                            
                    self.end_shape_beep()
                    self.post_shape()

                    done = True
                
            # ! if not in shapes list, draw it character per character
            if not done:
                for character in token.upper():

                    for coords in self.all_coords:
                        if character in coords.keys():
                            print("drawing ##### {} ####".format(character))
                            time.sleep(2)
                            char_points = coords[character]
                            initial_point = char_points[0]

                            if not (initial_point==self.coordinates).all():
                                while self.obstacle:
                                    print("obstacle!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
                                    continue
                                point_coordinates = np.array([initial_point['x'], initial_point['y']])
                                print("going to initial\n {}".format(point_coordinates))
                                #input()
                                angular_coords = self.get_angle_distance(point_coordinates)
                                self.move_to(angular_coords, dest_point=point_coordinates, simulate=False)
                                self.prev_step = {"vector": True, 'arc': [False, 0]}
                                self.update_position(point_coordinates)
                                self.update_prev_locations(point_coordinates)

                            # self.start_shape_beep()
                            self.clean_simulation()
                            for point in char_points[1:]:
                                while self.obstacle:
                                    print("obstacle!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
                                    continue
                                point_coordinates = np.array([point['x'], point['y']])
                        
                                if not (point_coordinates == self.coordinates).all():
                                    arc = False
                                    right = 0
                                    if not point['circular']:
                                        arc = False
                                        
                                        angular_coords = self.get_angle_distance(point_coordinates)
                                        print("####################################################")
                                        print("from {} to {}: {}".format(self.coordinates, point_coordinates, angular_coords))
                                        print("####################################################")
                                        #input()
                                        #! call to move
                                        self.move_to(angular_coords, dest_point=point_coordinates, simulate=True)
                                        # self.prev_step = {"vector": True, 'arc': [False, 0]}
                                    else:
                                        arc = True
                                        
                                        right = point['right']
                                        radius = self.get_radius(point_coordinates)
                                        # #print #("####################################################")
                                        # #print #("radius: {} to the right?: {} to {}".format(radius, right, point_coordinates))
                                        # #print #("####################################################")
                                        self.prev_step = {"vector": False, 'arc': [True, int(right)]}

                                        #! call to move
                                        self.move_circular(radius=radius, dest_point=point_coordinates,right=right)

                                    self.update_position(point_coordinates)
                                    self.update_prev_locations(point_coordinates)
                                    # #print #("new coord: {}\nprev_coords: {}".format(self.coordinates, self.previous_locations))
                                    # self.update_position(arc, right, point_coordinates)
                                    # self.prev_coords = np.copy(self.coordinates)
                                    # self.coordinates = np.array([point_coordinates[0], point_coordinates[1]])
                                    

                                # self.end_step_beep()
                                

                            
                                
                        else:
                            continue
                    self.end_shape_beep()
                    self.post_shape()

    def move_circular(self, radius, dest_point, right = True, simulate= True):
        #! calculate correction angle
        x_dest, y_dest = dest_point[0], dest_point[1]
        x_now, y_now = self.coordinates[0], self.coordinates[1]
        
        dest_vect = np.array([x_dest - x_now, y_dest - y_now])
        
        if (self.coordinates == self.prev_coords).all():
            prev_vect = np.array([1,0])
        else:
            prev_vect = np.array([x_now - self.prev_coords[0], y_now - self.prev_coords[1]])
                
        correction_angle = 0

        destination_vect_angle, counter_clock =  directed_angle(prev_vect, dest_vect, deg=True)
                
        if destination_vect_angle % 360 != 0:
            if counter_clock:
                correction_angle += destination_vect_angle    
            else:
                correction_angle -= destination_vect_angle
    
        #! at this point, we are facing the destination point
            
        if right == 1:
            correction_angle += 90
        elif right == -1:
            correction_angle -= 90
            
        #! here, the robot should be ready to start the circular path
        #! with the exeption that the angle is no correct because the previous path might be circular as well
            
        #! correcting the angle
            
        if self.prev_step['arc'][0]:
            if self.prev_step['arc'][1] == 1:
                correction_angle += 90
            elif self.prev_step['arc'][1] == -1:
                correction_angle -= 90

        # #print #("correction angle: {}".format(correction_angle))
        if correction_angle % 360 != 0:
            self.turn_left(correction_angle, deg=True)


        #! call to move
        self.circle(r=radius, angle=np.pi, right=right == 1, dest_point=dest_point, simulate=simulate)    

    def move_to(self, angular_coords, dest_point, simulate = False):
        
        angle = angular_coords[0]
        distance = angular_coords[1]
        
        if angle == 90:
            self.left_right_angle()
            pass
        elif angle == 180:
            self.left_right_angle()
            self.left_right_angle()
            pass
            
        else:
            print("turning by {} degs".format(angle))
            #input()
            if angle>0:
                self.turn_right(angle)
            elif angle<0:
                self.turn_left(angle)
        #! call to move
        
        self.forward(distance ,dest_point=dest_point, simulate=simulate)


    #* Function that run ros commands

    # ! pass negative distance to move backward
    def forward(self, distance=1, dest_point=None, simulate=True):
        if distance ==  0:
            distance = 1
        
        forward_time = distance  # / (self.speed)
        speed = self.speed_factor if distance > 0 else -1 * self.speed_factor

        command = """rosservice call /Motor 'rightspeed: {} \nleftspeed: {}'""".format(speed, speed)


        if dest_point is None:

            distance_from_origin = np.sqrt( self.coordinates[0] ** 2  + self.coordinates[1] ** 2)

            D = distance + distance_from_origin

            dest_point = ( D * np.cos(self.angle_with_x_ax), D * np.sin(self.angle_with_x_ax) )

        #if distance == 0:
        #    distance = np.sqrt( (self.coordinates[0] - dest_point[0])**2 +  (self.coordinates[1] - dest_point[1])**2)


        self.active_trajectory = straight_trajectory_equation(self.coordinates, dest_point, forward_time )

        self.active_timespace = np.linspace(0, forward_time, int(forward_time / (2 * 1e-3)) + 1)

        self.active_dest_point = dest_point

        self.active_method = 'forward'
        traj_msg = trajectory_msg()
        traj_msg.a1, traj_msg.a2, traj_msg.b1, traj_msg.b2 = self.coordinates[0], self.coordinates[1], dest_point[0], dest_point[1]
        traj_msg.t_f = forward_time
        traj_msg.straight_path = True
        traj_msg.right = False
        traj_msg.new = False
        traj_msg.pen_down = simulate

        self.publish_trajectory(traj_msg=traj_msg)
        # Move the robot for the specified forward_time
        subprocess.Popen(command, shell=True)
        time.sleep(forward_time)
        self.stop()
        return 0

    def backward(self, distance=1, dest_point=None, simulate=True):
        if distance == 0:
            distance = 1
        forward_time = distance #/ (self.speed)

        speed = -1 * self.speed_factor if distance > 0 else self.speed_factor

        command = """rosservice call /Motor 'rightspeed: {} \nleftspeed: {}'""".format(speed, speed)

        if dest_point is None:

            distance_from_origin = np.sqrt( self.coordinates[0] ** 2  + self.coordinates[1] ** 2)

            D = distance + distance_from_origin

            dest_point = ( D * np.cos(self.angle_with_x_ax), D * np.sin(self.angle_with_x_ax) )

        self.active_trajectory = straight_trajectory_equation(self.coordinates, dest_point, forward_time )

        self.active_timespace = np.linspace(0, forward_time, int(forward_time / (2 * self.delta_time)) + 1)

        traj_msg = trajectory_msg()

        traj_msg.a1, traj_msg.a2, traj_msg.b1, traj_msg.b2 = self.coordinates[0], self.coordinates[1], dest_point[0], dest_point[1]
        traj_msg.t_f = forward_time
        traj_msg.straight_path = True
        traj_msg.right = False
        traj_msg.new = False
        traj_msg.pen_down = simulate
        self.publish_trajectory(traj_msg=traj_msg)
        
        process = subprocess.Popen(command, shell=True)
        time.sleep(forward_time)
        
        self.stop()
        return 0

    def circle(self, r, angle = 2 * math.pi, right = True, dest_point=None, simulate=True):
        s = r * angle
        
        ext_motor_speed = self.speed
        int_motor_speed = ext_motor_speed * (1 - (self.wheel_base/r)) if r != self.wheel_base\
            else ext_motor_speed
        
        ext_motor_speed = (ext_motor_speed / self.speed) * self.speed_factor
        int_motor_speed = (int_motor_speed / self.speed) * self.speed_factor

        forward_time = s/self.speed

       
        if right:
            command = """rosservice call /Motor 'rightspeed: {} \nleftspeed: {}'""".format(int_motor_speed, ext_motor_speed)
        else:
            command = """rosservice call /Motor 'rightspeed: {} \nleftspeed: {}'""".format(ext_motor_speed, int_motor_speed)

        self.active_trajectory = circular_trajectory_equations(speed=self.speed, current_point=self.coordinates, dest_point=dest_point, t_f=forward_time,radius=r, clockwise=right)

        self.active_timespace = np.linspace(0, forward_time, int(forward_time / (2 * self.delta_time)) + 1)

        traj_msg = trajectory_msg()

        traj_msg.a1, traj_msg.a2, traj_msg.b1, traj_msg.b2 = self.coordinates[0], self.coordinates[1], dest_point[0], dest_point[1]
        traj_msg.t_f = forward_time
        traj_msg.straight_path = False
        traj_msg.right = right
        traj_msg.new = False
        traj_msg.radius=r
        traj_msg.pen_down = simulate
        self.publish_trajectory(traj_msg=traj_msg)
        
        #print #("moving in a circular path, radius: {}".format(r))

        process = subprocess.Popen(command, shell=True)
        time.sleep(forward_time)
        
        self.stop()
        return 0
    
    def square(self, side_length, right):
        forward_time = side_length / (self.speed)
        theta = math.pi / 2
        turn_time = (theta * self.wheel_base) / (self.speed)

        for side_ in range(4):
            #print #("side: {} for {} seconds".format(side_, forward_time))

            self.forward(side_length)

            time.sleep(forward_time)

            self.stop()

            if right:
                self.turn_right(math.pi)
                self.turn_right()

            else:
                self.turn_left(math.pi/2)



        return 0

    # def post_character(self):
    #     angular_coords = self.get_angle_distance([0, 0])
    #     print("from: {} to [0, 0]: {}".format(self.coordinates, angular_coords))
    #     self.move_to(angular_coords, None, simulate=False)

    #########################################
    #!###############################################################


    #! functions that moves the robot without changing its coordinates, i.e: turns
    #!###############################################################

    def turn(self, theta, right=True, deg = True):

        if theta < 0 :
            if right:
                self.turn_left(np.abs(theta))
            else:
                self.turn_right(np.abs(theta))
        if not deg: 
            theta = np.degrees(theta) % 360
        else:
            turn_radius = (self.wheel_base / 2)
            arc_length = (np.radians(theta) * turn_radius)
            forward_time = float(  (arc_length / (self.speed )))
            r_speed = -1*self.speed_factor if right else self.speed_factor
            l_speed = self.speed_factor if right else -1*self.speed_factor 
            command = """rosservice call /Motor 'rightspeed: {} \nleftspeed: {}'""".format(r_speed, l_speed)
            process  = subprocess.Popen(command, shell=True)
            #!###############################################
            time.sleep(forward_time)
            #!###############################################
            self.stop()
            self.angle_with_x_ax = (self.angle_with_x_ax + theta) if not right else (self.angle_with_x_ax - theta)
            self.angle_with_x_ax %= 360
            rospy.logwarn("angle with x vector: {}".format(self.angle_with_x_ax))
        return 0
    
    def turn_left(self, theta, deg = True):
        if theta < 0 :
            self.turn_right(np.abs(theta))

        if not deg: 
            theta = np.degrees(theta) % 360
        
            
        else:
            turn_radius = (self.wheel_base / 2)
 
            arc_length =np.abs(np.radians(theta) * turn_radius)
            print("theta: ", theta)
            print("theta rads: ", np.radians(theta))
            print("arc_length: ", arc_length)

            forward_time = float(  (arc_length / (self.speed )))

            #print #("turning left, angle : {}, time : {}".format(theta, forward_time))

            command = """rosservice call /Motor 'rightspeed: {} \nleftspeed: {}'""".format(self.speed_factor, -1*self.speed_factor)
            process  = subprocess.Popen(command, shell=True)

            #!###############################################
            time.sleep(forward_time)
            #!###############################################
            self.stop()
            self.angle_with_x_ax = (self.angle_with_x_ax + theta) % 360
            rospy.logwarn("angle with x vector: {}".format(self.angle_with_x_ax))
            #input()
        return 0
     
    def left_right_angle(self):
        #print #("turning left, angle : 90, time : {}".format(self.right_angle_time))
        command = """rosservice call /Motor 'rightspeed: {} \nleftspeed: {}'""".format(self.speed_factor, -1*self.speed_factor)
        process = subprocess.Popcess = subprocess.Popen(command, shell=True)
        time.sleep(self.right_angle_time)
        self.stop()
        self.angle_with_x_ax = (self.angle_with_x_ax + 90) % 360
        rospy.logwarn("angle with x vector: {}".format(self.angle_with_x_ax))
        #input()
        return 0
    
    def right_right_angle(self):
        #print #("turning right, angle : 90, time : {}".format(self.right_angle_time))
        command = """rosservice call /Motor 'rightspeed: {} \nleftspeed: {}'""".format(-1*self.speed_factor, self.speed_factor)
        process = subprocess.Popen(command, shell=True)
        time.sleep(self.right_angle_time)
        self.stop()
        self.angle_with_x_ax = (self.angle_with_x_ax - 90) % 360
        rospy.logwarn("angle with x vector: {}".format(self.angle_with_x_ax))
        #input()
        return 0

    def turn_right(self, theta, deg = True):
        #print #("turning right by {}".format(theta))
        if theta < 0 :
            self.turn_left(np.abs(theta))
            
        if not deg: 
            theta = np.degrees(theta) % 360
        

        else:
            turn_radius = (self.wheel_base / 2)

            arc_length = (np.radians(theta) * turn_radius)

            forward_time = float(  (arc_length / (self.speed )))
            
            command = """rosservice call /Motor 'rightspeed: {} \nleftspeed: {}'""".format(-1*self.speed_factor, self.speed_factor)
            process = subprocess.Popen(command, shell=True)
            time.sleep(forward_time)
            self.stop()
            self.angle_with_x_ax = (self.angle_with_x_ax - theta) % 360
            rospy.logwarn("angle with x vector: {}".format(self.angle_with_x_ax))
            #input()
        return 0

    #!###############################################################


    #! fuctions for the buzzer
    #!###############################################################
    def start_shape_beep(self):
        self.buzzer(2)

    def end_step_beep(self):
        self.buzzer(0.5)

    def end_shape_beep(self):
        subprocess.Popen('printf "\a"', shell=True)
        time.sleep(3)
        self.buzzer(0.1)
        time.sleep(0.3)
        self.buzzer(0.1)

    def buzzer(self, t):
        command = """rosservice call /Buzzer 'buzzer: 1'"""
        command_stp = """rosservice call /Buzzer 'buzzer: 0'"""
        process = subprocess.Popen(command, shell=True)
        time.sleep(t)
        process = subprocess.Popen(command_stp, shell=True)

    def post_shape(self):
        
        angular_coords = self.get_angle_distance([0, 0])
        self.move_to(angular_coords, [0, 0], simulate=False)
        
        


    #!###############################################################


    #! to stop the robot
    #!###############################################################

    def stop(self):
        rospy.loginfo("stopping")
        command = """rosservice call /Motor 'rightspeed: 0 \nleftspeed: 0'"""
        subprocess.call(command, shell=True)
        return 0
        
    #!###############################################################
    





    
    def t_d(self, d):
        return d
      
    def d_t(self, t):
        return t
    

    
    def get_angle_distance(self, coordinates):
        
        x, y = coordinates[0], coordinates[1]

        if(coordinates == self.coordinates).all():
            return (0., 0.)
        delta_x = x - self.coordinates[0]
        delta_y = y - self.coordinates[1]
        #print #(x, y)
        


        destination_vector = np.array([delta_x, delta_y])

        if (self.prev_coords == self.coordinates).all():
            previous_vector = np.array([1, 0])
        else:
            previous_vector = np.array([self.coordinates[0] - self.prev_coords[0], self.coordinates[1] - self.prev_coords[1]])

        # #print #(previous_vector, destination_vector)
        magnitude_destination_vect = np.sqrt( np.sum([destination_vector[0]**2, destination_vector[1]**2] ) )

        angle = angle_vectors(source=np.array([1, 0]), destination=destination_vector)

        angle = (self.angle_with_x_ax - angle) % 360

        rospy.logwarn("angular coords: {}".format((angle ,magnitude_destination_vect)))
        #input()

        # angle = angle_vectors(previous_vector, destination_vector)

        return (float("%.4f" % angle), float("%.4f" % (magnitude_destination_vect * self.distance_factor)))

    def get_radius(self, dest_point):

        x_now, y_now = self.coordinates[0], self.coordinates[1]
        x_dest, y_dest = dest_point[0], dest_point[1]
        distance_destination = np.sqrt( (x_dest - x_now)**2 + (y_dest - y_now)**2 )

        return float("%.2f"%((distance_destination / 2) ))
    



    def update_position(self, new_coords):
        self.coordinates = new_coords
        return True

    def re_init_coord(self):
        self.prev_coords = self.coordinates
        self.coordinates = np.array[0, 0]
        return






#! these mesurments are not correct!
#! retake them when possible
            
#! find a solution to calculate the correction angle after a circular path
# controller = Rob<otController(0.5, 0.5, 0.13, node_name='primary_robot')

# controller.draw('S')


# controller.turn_right(90)

# controller.forward(0.5)

# controller.backward(0.5)

# controller.square(0.3, True)

# controller.circle(0.3, True)

# controller.stop()
        


# command = """rosservice call /Motor 'rightspeed: 0\nleftspeed: 0'"""
# process = subprocess.Popen(command, shell=True)
# process.wait()

# #print #(angle_vectors(np.array([1, -2]), np.array([-0.5, 1])))

def main():
    

    node_name = "primary_robot"
    rospy.init_node(node_name)
    controller = RobotController(0.5, 0.5, 0.13, node_name='primary_robot')

    # controller.coordinates = [1, 1]
    # controller.angle_with_x_ax = (45 * np.pi) / 180
    # controller.circle(1, 3.14,True, dest_point=[2,0], simulate=True)
    # controller.forward(1, [1, 0], True)
    # controller.clean_simulation()
    controller.draw("BC square")
    # controller.stop()


    # traj_msg = trajectory_msg()

    return
    traj_msg.a1, traj_msg.a2, traj_msg.b1, traj_msg.b2 = 0., 0., 0., 0.
    traj_msg.t_f = 1e-3
    traj_msg.straight_path = True
    traj_msg.right = False
    traj_msg.new = True
    rospy.loginfo("seding: \n{}".format(traj_msg))
    pub = rospy.Publisher(controller.trajectory_topic_name, trajectory_msg, queue_size=10)
    pub.publish(traj_msg)
    start = [[0, 0], [0, 0], [1, 2], [2, 0] , [1.5, 1], [0.5, 1] ]
    dest =  [[0, 0], [1, 2], [2, 0], [1.5, 1], [0.5, 1], [0, 0] ]

    new = True
    for s, d in zip(start, dest):
        publisher = rospy.Publisher("trajectory", trajectory_msg, queue_size=10)
        #print #("publishing")
        msg = trajectory_msg()
        msg.a1, msg.a2, msg.b1, msg.b2 = s[0], s[1], d[0], d[1]
        msg.t_f = 2
        msg.straight_path = True
        msg.right = True
        msg.new = new
        if new:
            new = False
        publisher.publish(msg)
        #print #("published")
        time.sleep(2)



    new = True
    for s, d in zip(start, dest):
        publisher = rospy.Publisher("trajectory", trajectory_msg, queue_size=10)
        #print #("publishing")
        msg = trajectory_msg()
        msg.a1, msg.a2, msg.b1, msg.b2 = s[0], s[1], d[0], d[1]
        msg.t_f = 2
        msg.straight_path = True
        msg.right = True
        msg.new = new
        publisher.publish(msg)
        #print #("published")
        if new:
            new = False
        time.sleep(2)
    # controller.test_pub()


    # start = [[0,0], [1,2]]
    # dest = [[1,2], [2, 0]]
    # for s, d in zip(start, dest):
    #     publisher = rospy.Publisher("trajectory", trajectory_msg, queue_size=10)
    #     #print #("publishing")
    #     msg = trajectory_msg()
    #     msg.a1, msg.a2, msg.b1, msg.b2 = s[0], s[1], d[0], d[1]
    #     msg.t_f = 2
    #     msg.straight_path = True
    #     msg.right = True
    #     publisher.publish(msg)
    #     #print #("published")
    #     time.sleep(3)
    # #print #("here")
    # rate = rospy.Rate(0.1)
    # # controller.buzzer(1)
    # controller.forward(1, (2,1))
    # cmd = {'action': 'move in the shape', 'unit': '', 'value': 0., 'subject': 'C'}
    # cmd = {'action': 'move in a straight line', 'unit': '', 'value': 10, 'subject': ''}

    # # cmd = {'action': 'make a turn', 'unit': 'd', 'value': 20., 'subject': 'right'}
    # command = Commands_msg()
    # command.action = cmd.get("action")
    # command.subject = cmd.get("subject")
    # command.value = cmd.get("value")
    # command.unit = cmd.get("unit")

    # #print #("command: {}".format(cmd))


    # # position_msg = Position_msg()
    # # position_msg.x = 0.
    # # position_msg.y = 0.

    # # controller.forward(1)

    # controller.draw("A")
    # # controller.stop()
    # # controller.buzzer(0)

    # while not rospy.is_shutdown():
    #     #print #(controller.position_topic.publish(controller.node_name, position_msg))
    #     rate.sleep()

if __name__ == "__main__":
    
    
    try:
        main()
    except rospy.ROSInterruptException:
        pass




"""

TODO: define the s_robot class and the p_robot class

! IMPORTANT: still don't know how to control several robots, how to organize topics
? suggestions: 
    * create a service for each robot, and change the command building process in the Controller class to adjust the service according to the situation     

TODO: initialize the list of secondary robots based on their number, i.e: if there are 3 robots, then s_robot_1, s_robot_2 and s_robot_3, from the secondary robot class

"""

# TODO
#! map commands to functions

#! animate robot position ploting in real time
    #? extract the equation of the circle and use it to get the point in a delta time