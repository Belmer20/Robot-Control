import subprocess
import time
import math
from coords import Alphabet
import numpy as np
import os
from utils import angle_vectors, directed_angle
from Topics.Position import Position
from std_msgs.msg import String
from jetbot_mini.msg import Position_msg
import rospy

# os.chdir(os.getcwd() + "/robot")

class RobotController(object):
    def __init__(self, full_speed, speed_factor, wheel_base, node_name, new_subs = []):
        # self.robot = robot
        
        self.wheel_base = wheel_base
        self.full_speed = full_speed
        self.speed_factor = speed_factor
        self.speed = self.speed_factor * self.full_speed
        self.coordinates = np.array([0, 0])
        self.prev_coords = np.array([0, 0])
        self.prev_step = {"vector": False, "arc": [False, 0]}
        self.right_angle_time = 0.5
        self.distance_factor=0.5
        
        self.alphabet_coords = Alphabet().alphabet
        self.state = False
        # if not(len(new_subs) == 0) :
        #     self.position_subscribes = self.subscribe_pos(new_subs, node_name)
        # else:
        #     self.position_subscribes = []

    # def sub_to_others_positions(self, number_others):
    #     others = [{"index": s, "callback": self.update_others_position} for s in range(number_others) if not (s == self.index)]
    #     self.others_positions = [{"index": s, "x": s, "y": 0} for s in range(number_others) if not (s == self.index)]
    #     self.subscribe_pos(others)
    #     return [s for s in range(number_others) if not (s == self.index)]

    def update_others_position(self, data):
        rospy.loginfo(data)

    def subscribe_pos(self, new_subs):
        for sub in new_subs:
            #! new_subs is a list with the callbacks functions for every new subscriber
            topic_name = "position_R_"+str(sub.get("index"))
            print("subscribing to {}".format(topic_name))
            subscriber = self.position_topic.get_subscriber(topic_name=topic_name, callback=sub.get("callback"))
            self.position_subscribes.append({"topic_name":topic_name, "subscriber": subscriber})


    def stop(self):
        command = """rosservice call /Motor 'rightspeed: 0 \nleftspeed: 0'"""
        subprocess.call(command, shell=True)
        return 0
        
    def turn_left(self, theta, deg = True):
        if not deg: 
            theta = np.degrees(theta)
        if theta < 0:
            for process in self.turn_right(np.abs(theta)):
                process.wait()
            
        else:
            turn_radius = (self.wheel_base / 2)
 
            arc_length = (np.radians(theta) * turn_radius)

            forward_time = float(  (arc_length / (self.speed )))

            print("turning left, angle : {}, time : {}".format(theta, forward_time))

            command = """rosservice call /Motor 'rightspeed: {} \nleftspeed: {}'""".format(self.speed_factor, -1*self.speed_factor)
            process  = subprocess.Popen(command, shell=True)
            time.sleep(0.5*forward_time)
            
            process.wait()
            self.stop()
        return 0
     
    def left_right_angle(self):
        print("turning left, angle : 90, time : {}".format(self.right_angle_time))
        command = """rosservice call /Motor 'rightspeed: {} \nleftspeed: {}'""".format(self.speed_factor, -1*self.speed_factor)
        process = subprocess.Popen(command, shell=True)
        time.sleep(self.right_angle_time)
        process.wait()
        self.stop()
        return 0
    
    def right_right_angle(self):
        print("turning right, angle : 90, time : {}".format(self.right_angle_time))
        command = """rosservice call /Motor 'rightspeed: {} \nleftspeed: {}'""".format(-1*self.speed_factor, self.speed_factor)
        process = subprocess.Popen(command, shell=True)
        time.sleep(self.right_angle_time)
        process.wait()
        self.stop()
        return 0

    def turn_right(self, theta, deg = True):
        print("turning right by {}".format(theta))
        if not deg: 
            theta = np.degrees(theta)
        if theta < 0:
            for process in self.turn_left(np.abs(theta)):
                process.wait()

        else:
            turn_radius = (self.wheel_base / 2)

            arc_length = (np.radians(theta) * turn_radius)

            forward_time = float(  (arc_length / (self.speed )))
            
            command = """rosservice call /Motor 'rightspeed: {} \nleftspeed: {}'""".format(-1*self.speed_factor, self.speed_factor)
            process = subprocess.Popen(command, shell=True)
            time.sleep(0.5*forward_time)
            process.wait()
            self.stop()
        return 0

    def forward(self, distance):

        forward_time = distance #/ (self.speed)

        print("moving forward, time : {}".format(forward_time))

        command = """rosservice call /Motor 'rightspeed: {} \nleftspeed: {}'""".format(self.speed_factor, self.speed_factor)

        process = subprocess.Popen(command, shell=True)
        time.sleep(forward_time)
        process.wait()

        self.stop()
        return 0

    def backward(self, distance):
        forward_time = distance #/ (self.speed)

        command = """rosservice call /Motor 'rightspeed: {} \nleftspeed: {}'""".format(-1*self.speed_factor, -1*self.speed_factor)
        
        process = subprocess.Popen(command, shell=True)
        time.sleep(forward_time)
        process.wait()
        self.stop()
        return 0

    def square(self, side_length, right):
        forward_time = side_length / (self.speed)
        theta = math.pi / 2
        turn_time = (theta * self.wheel_base) / (self.speed)

        for side_ in range(4):
            print("side: {} for {} seconds".format(side_, forward_time))

            self.forward(side_length)

            time.sleep(forward_time)

            self.stop()

            if right:
                self.turn_right(math.pi)
                self.turn_right()

            else:
                for process in self.turn_left(math.pi/2):
                    process.wait()

            time.sleep(1)
            
        return 0

    def circle(self, r, angle = 2 * math.pi, right = True):
        s = r * angle
        
        ext_motor_speed = self.speed
        int_motor_speed = ext_motor_speed * (1 - (self.wheel_base/r)) if r != self.wheel_base\
            else ext_motor_speed
        
        ext_motor_speed = (ext_motor_speed / self.speed) * self.speed_factor
        int_motor_speed = (int_motor_speed / self.speed) * self.speed_factor

        time_needed = s/self.speed

       
        if right:
            command = """rosservice call /Motor 'rightspeed: {} \nleftspeed: {}'""".format(int_motor_speed, ext_motor_speed)
        else:
            command = """rosservice call /Motor 'rightspeed: {} \nleftspeed: {}'""".format(ext_motor_speed, int_motor_speed)

        print("moving in a circular path, radius: {}".format(r))

        process = subprocess.Popen(command, shell=True)
        time.sleep(time_needed)
        process.wait()
        self.stop()
        return 0
    
    def t_d(self, d):
        return d
      
    def d_t(self, t):
        return t
    
    def buzzer(self, t):
        command = """rosservice call /Buzzer 'buzzer: 1'"""
        command_stp = """rosservice call /Buzzer 'buzzer: 0'"""
        process = subprocess.Popen(command, shell=True)
        time.sleep(t)
        process = subprocess.Popen(command_stp, shell=True)
        process.wait()
    
    def get_angle_distance(self, coordiates):
        
        x, y = coordiates
        delta_x = x - self.coordinates[0]
        delta_y = y - self.coordinates[1]

        destination_vector = np.array([delta_x, delta_y])
        if (self.prev_coords == self.coordinates).all():
            previous_vector = np.array([1, 0])
        else:
            previous_vector = np.array([self.coordinates[0] - self.prev_coords[0], self.coordinates[1] - self.prev_coords[1]])

        magnitude_destination_vect = np.sqrt( np.sum([destination_vector[0]**2, destination_vector[1]**2] ) )
        angle = angle_vectors(previous_vector, destination_vector)

        return (float("%.2f" % angle), float("%.2f" % (magnitude_destination_vect * self.distance_factor)))
    
    def move_to(self, angular_coords):
        
        angle = angular_coords[0]
        distance = angular_coords[1]
        
        if angle == 90:
            self.left_right_angle()
        elif angle == 180:
            self.left_right_angle()
            self.left_right_angle()
            
        else:
            print("turning left by {} degs".format(angle))
            for process in self.turn_left(angle):
                process.wait()

        self.forward(distance)

    def move_circular(self, radius, dest_point, right = True):
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

        print("correction angle: {}".format(correction_angle))
        if correction_angle % 360 != 0:
            self.turn_left(correction_angle, deg=True)



        self.circle(r=radius, angle=np.pi/1.4, right=right == 1)      

    def get_radius(self, dest_point):

        x_now, y_now = self.coordinates[0], self.coordinates[1]
        x_dest, y_dest = dest_point[0], dest_point[1]
        distance_destination = np.sqrt( (x_dest - x_now)**2 + (y_dest - y_now)**2 )

        return float("%.2f"%((distance_destination / 2) * self.distance_factor))
    
    def start_shape_beep(self):
        self.buzzer(2)

    def end_step_beep(self):
        self.buzzer(0.5)

    def end_shape_beep(self):
        time.sleep(3)
        self.buzzer(0.1)
        time.sleep(0.3)
        self.buzzer(0.1)

    def post_character(self):
        angular_coords = self.get_angle_distance([0, 0])
        print("from: {} to [0, 0]: {}".format(self.coordinates, angular_coords))
        self.move_to(angular_coords)
    
    # def update_position(self):
    #     position_message = "new position"
    #     if self.position_publisher:
    #         self.position_topic.publish(self.node_name, position_message)
    #     else:
    #         rospy.logerr("Failed to publish position message. Publisher not available.")

    def re_init_coord(self):
        self.prev_coords = self.coordinates
        self.coordinates = np.array[0, 0]
        return

    def write(self, text):
        text = text.upper()

        for character in text:
            if character in self.alphabet_coords.keys():
                char_points = self.alphabet_coords[character]
                initial_point = char_points[0]
                if not (initial_point==self.coordinates).all():
                    point_coordinates = np.array([initial_point['x'], initial_point['y']])
                    angular_coords = self.get_angle_distance(point_coordinates)
                    self.move_to(angular_coords)
                    self.prev_step = {"vector": True, 'arc': [False, 0]}

                self.start_shape_beep()

                for point in char_points[1:]:

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

                            
                            self.move_to(angular_coords)
                            # self.prev_step = {"vector": True, 'arc': [False, 0]}
                        else:
                            arc = True
                            
                            right = point['right']
                            radius = self.get_radius(point_coordinates)
                            print("####################################################")
                            print("radius: {} to the right?: {} to {}".format(radius, right, point_coordinates))
                            self.move_circular(radius=radius, dest_point=point_coordinates,right=right)
                            print("####################################################")
                            # self.prev_step = {"vector": False, 'arc': [True, int(right)]}

                        self.update_position(arc, right, point_coordinates)
                        # self.prev_coords = np.copy(self.coordinates)
                        # self.coordinates = np.array([point_coordinates[0], point_coordinates[1]])

                    self.end_step_beep()
                    

                self.end_shape_beep()

                self.post_shape()
                    
            else:
                continue






#! these mesurments are not correct!
#! retake them when possible
            
#! find a solution to calculate the correction angle after a circular path
# controller = Rob<otController(0.5, 0.5, 0.13, node_name='primary_robot')

# controller.write('S')


# controller.turn_right(90)

# controller.forward(0.5)

# controller.backward(0.5)

# controller.square(0.3, True)

# controller.circle(0.3, True)

# controller.stop()
        


# command = """rosservice call /Motor 'rightspeed: 0\nleftspeed: 0'"""
# process = subprocess.Popen(command, shell=True)
# process.wait()

# print(angle_vectors(np.array([1, -2]), np.array([-0.5, 1])))

def main():
    node_name = "primary_robot"

    rospy.init_node(node_name, anonymous=True)
    rate = rospy.Rate(0.1)
    controller = RobotController(0.5, 0.5, 0.13, node_name='primary_robot')
    position_msg = Position_msg()
    position_msg.x = 0.
    position_msg.y = 0.

    while not rospy.is_shutdown():
        print(controller.position_topic.publish(controller.node_name, position_msg))
        rate.sleep()

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