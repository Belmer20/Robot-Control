#!/usr/bin/env python

from utils import get_robot_conf, Queue    # Changed LIFO to FIFO
from threading import Thread
from AI_Driver import AI_Driver
from jetbot_mini.msg import Obstacle_msg
from Topics.Obstacle import Obstacle
import numpy as np
import rospy
import cv2

"""
    #! this class is responsible for the Camera of the robot
    
    #? Main Responsibilities
        #* get live feed from the camera ->  

        #* pass frames to the object detection model and get object boxes

        #* use the Distance estimator class to estimate the distance from the object
            # the DistanceEstimator class is only responsible estimating the distance

        #* send emergency stop signals to the obstacle topic based on the estimated distance and a predefined threshold
        
        #? Optional
        #* display the live feed, stream it if possible 


"""


"""
    #! Working logic

    # main thread responsible for capturing frames and putting them in the frames queue

    # another thread estimates the distance while the queue is not empty

    # if the distance is less than the threshold, a stop signal is sent to the robot via the obstacle topic
      
"""

class CollisionDetector:

    #__instance = None

    #def __new__(cls, *args, **kwargs):
     #   if cls.__instance is None:
      #      cls.__instance = super(AI_Driver, cls).__new__(cls)
       #     cls.__instance.__initialized = False
#        return cls.__instance

    def __init__(self, node_name):
    #if not self.__initialized:
        robot_conf = get_robot_conf()

        if not (robot_conf["index"] == 0):
            rospy.logerr("NOT A PRIMARY ROBOT NODE")
            raise Exception("this is a Primary node, check the conf.json file and make sure that the index is 0")

        conf_camera_name = node_name if robot_conf['camera_node_name'] is None else robot_conf['camera_node_name']
        # number_others = robot_conf["number_others"]
        # self.master_name = robot_conf["master_name"]
        self.index = robot_conf["index"]
        self.node_name = conf_camera_name + str(self.index)
        self.init()

        self.rate = rospy.Rate(30)  # 30hz <=> 30 frames per second

        self.obstacle_topic = Obstacle()
        self.obstacle_topic.set_name("obstacle_" + self.node_name)
        self.obstacle_topic.register_node(self.node_name)

        """
            #zdzzzddzffezrrrrdrzdzrdtfdfftftstdz-tdz-tzdz-tdz-tdztd-ztdz-t-td-yt-tz-tzttztztydztydtdddtztfdztftzczdfztzfttrccccccccccctsssstyyyyyyyyyddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddd-----------------------------------------------------uuuuuuuuuuuuuuuuytyu-UserWarningdddddddddddddddddduu------------u------------------------------------------------------------------------------------------------------------------zzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzz--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------zzzzzzzzzzzzzzzzzzzzzzzzzzzzzzz-zzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuu-
            #ucvcdddddddddddddddddddddddddddddddddddddddddddddddddddddddddacdusddddddddddddddddddddddddddddddddddddddddddsssssssssssssssssssssssssssssssssssssssssssssssssssssssssfuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuu
        """
        self.camera_frame_size = (320, 320)

        self.frames_queue_size = 120
            # self.frames_queue = LIFO(self.frames_queue_size, np.ndarray)  # Changed LIFO to FIFO
        self.frames_queue = Queue(self.frames_queue_size, np.ndarray, mode="LIFO")
        self.active_frame = np.array([])

            # ! explanation:
            #? based on our implementation, the estimation is done for three cases
            #    * the object is centered in the frame (i.e: facing the robot) => delta_r and delta_l are compared to c_threshold
            #    * the object is to the left of the robot => delta_r is compared to r_threshold
            #    * the object is to the right of the robot => delta_l is compared to l_threshold

        self.c_threshold = int((20 * self.camera_frame_size[0]) / 100)
        self.r_threshold = int((80 * self.camera_frame_size[0]) / 100)
        self.l_threshold = int((80 * self.camera_frame_size[0]) / 100)

        self.distance_thread = Thread(target=self.analyse_frames)
        self.distance_thread.daemon = True

            # self.distance_estimator = DistanceEstimator()
        self.object_boxer = AI_Driver().obstacle_detection

        self.distance_thread.start()
        self.frames_feed()




    def init(self):
        rospy.loginfo("starting node: {}".format(self.node_name))
        rospy.init_node(self.node_name, anonymous=False)

    def emergency_stop(self, stop):
        # TODO
        # ! send stop signal

        msg = Obstacle_msg()

        msg.obstacle = stop
        self.obstacle_topic.publish(self.node_name, msg)
        rospy.logwarn("published obstacle: {}.".format(stop))

    def detect_collision(self, box_coords):
        """
        # ? box_coords is a list of dictionaries containing boxes of objects
        # ? dictionary keys:
            # * left the most left pixel of the object
            # * right the most right pixel of the object
            # * top the most top pixel of the object
            # * bottom the most bottom pixel of the object
        """
        for box in box_coords:

            delta_left = box["left"]

            delta_right = box["right"]

            delta_left = int(delta_left)

            delta_right = int(self.camera_frame_size[0] - delta_right)

            rospy.loginfo("\ndelta_right: {}\ndelta_left: {}".format(delta_right, delta_left))

            if delta_left > 0 and delta_right > 0:

                if (delta_left <= self.c_threshold and delta_right <= self.c_threshold):
                    self.emergency_stop(True)
                    return
                    
                elif delta_left <= 0:
                    if delta_right <= self.r_threshold:
                        self.emergency_stop(True)
                        return

                elif delta_right <= 0:
                    if delta_left <= self.l_threshold:
                        self.emergency_stop(True)
                        return

                self.emergency_stop(False)

    def analyse_frames(self):
        while True:
            while not self.frames_queue.is_empty():
                self.active_frame = self.frames_queue.dequeue()  # Changed from pop() to dequeue()

                # ! this function calls the model, and returns the box containing the object
                objects_boxes = self.object_boxer(frames=[self.active_frame])

                # ! this function does the distance estimation
                if objects_boxes:
                    self.detect_collision(box_coords=objects_boxes)
                else:
                    rospy.logerr("no boxes")
                    rospy.loginfo("active frame:\n{}".format(self.active_frame))
                self.rate.sleep()

    def frames_feed(self):
        cap = cv2.VideoCapture(0)
        while True:
            # rospy.loginfo("getting frame")

            # !##############################
            # TODO:

            while cap.isOpened():
                ret, frame = cap.read()
                if not ret:
                    break

                cv2.imshow("Frame: ", frame)

                frame = np.array(frame)

                frame = cv2.resize(frame, (320, 320))

                rospy.loginfo("pushed to fifo: {}".format(self.frames_queue.enqueue(frame)))  # Changed from push() to enqueue()

                # Break the loop when 'q' key is pressed
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

                self.rate.sleep()

            cap.release()
            cv2.destroyAllWindows()

            # logic to get the frames from the camera
            # !##############################
            # frame = np.array(frame)
            # frame = cv2.resize(frame, (320, 320))
            # print("shape: ",frame.shape)
            # self.frames_queue.enqueue(frame)  # Changed from push() to enqueue()

def main():
    node_name = "Camera_"
    collisionDetector = CollisionDetector(node_name)

    # rospy.loginfo("{} node started.".format(node_name))

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass