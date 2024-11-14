#!/home/alpha/Desktop/Project-2CS/robot/ai_py_env/bin/python3.10

from jetbot_mini.msg import State_msg, Commands_msg, Command_classes_msg
from CollisionDetector import CollisionDetector
from fastapi import FastAPI, UploadFile, File
from CommandsDecoder import CommandsDecoder
from uvicorn import run as uvicorn_run
from audio_utils import extract_mfcc
# from AI_Driver import AI_Driver
from utils import check_float
from threading import Thread
from fastapi import FastAPI
from typing import List
import numpy as np
import tempfile
import librosa
import rospy
import time
import cv2
import os



ready_to_receive = True

rospy.init_node("API")
app = FastAPI()
# ai_driver = AI_Driver()



class Communicator:
    def __init__(self):
        self.master_state_topic = "master_state"
        self.master_commands_topic = "master_commands"
        self.master_commands_classes_topic = "master_commands_classes"
        self.commands_publisher = rospy.Publisher(self.master_commands_topic, Commands_msg, queue_size=10)
        self.command_classes_publisher = rospy.Publisher(self.master_commands_classes_topic, Command_classes_msg, queue_size=10)


    def publish_user_command(self, decoded_cmd):
        """
        publish the command object to corespondig Topics
        returns True if success
        """

        rospy.loginfo("publishing: \n{}".format(decoded_cmd["cmd_obj"]))
        self.commands_publisher.publish(decoded_cmd["cmd_obj"])

        print('publilshed')
        print(decoded_cmd["cmd_obj"])
        
        input()


        # rospy.loginfo("publishing: \n{}".format(decoded_cmd["cmd_classes"]))
        # input()
        self.command_classes_publisher.publish(decoded_cmd["cmd_classes"])

        return True

class Decoder:
    """
        #!Interface with the commands decoder
    """
    def __init__(self):
        self.decoder = CommandsDecoder()

    def decode_cmd(self, text_command):
        """
            decodes the text command
            return command and command classes objects
        """
        command_tuple = self.decoder.label_command(text_command)
        command_obj = command_tuple[0]
        command_classes_ = command_tuple[1]

        command = Commands_msg()
        command_classes = Command_classes_msg()

        command.action = command_obj.get('action')
        command.subject = command_obj.get('subject')
        command.value = float(command_obj.get('value')) if check_float(command_obj.get('value')) else 0.
        command.unit = command_obj.get('unit')


        command_classes.action = str(command_classes_.get('action'))
        command_classes.subject = str(command_classes_.get('subject'))
        command_classes.value = str(command_classes_.get('value'))
        command_classes.unit = str(command_classes_.get('unit'))

        return  {"cmd_obj": command, "cmd_classes": command_classes}

class Response:
    def __init__(self, success, message = "", data = None):
        self.success = success
        self.message = message
        self.data = data

        self.body = self.body_()

    def body_(self):
        return {"success": self.success, "message": self.message, "data": self.data}


decoder = Decoder()
communicator = Communicator()


def change_ready_state(data):
    global ready_to_receive
    ready_to_receive = data.ready
    print("###################\nstate updated to {}\n###################\n".format(ready_to_receive))

def monitor_state():
    rospy.Subscriber(communicator.master_state_topic, State_msg, change_ready_state)
    rospy.spin()

def state_monitor_thread():
    thread = Thread(target=monitor_state, args=())
    thread.daemon = True
    thread.start()
    return thread

state_thread = state_monitor_thread()


@app.get("/ready_to_receive")
def check_if_ready():
    return Response(True, "Hello from jetbot_mini API", ready_to_receive)

@app.get("/switch_mode")
def switch_mode():
    global ai_driver
    ai_driver.switch_mode()

    return Response(True, "mode switched to {}".format("signs" if ai_driver.mode else "speech"), ai_driver.mode).body

@app.post("/set_mode")
def set_mode(mode):
    mode = int(mode)
    if (mode in [0, 1]):
        if not (ai_driver.mode == mode):
            ai_driver.switch_mode()
        return Response(True, "mode switched to {}".format("signs" if ai_driver.mode else "speech"), ai_driver.mode).body
    else:
        return Response(False, "mode should be 0 for speech or 1 for signs").body    

@app.post("/command/")
def command_router(text_cmd: str):
    
    # command_dict = CommandsDecoder().label_command(text_cmd)[0]
    decoded_command = decoder.decode_cmd(text_cmd)

    print(decoded_command)

    input()

    communicator.publish_user_command(decoded_command)

    # command = Commands_msg()

    # command.action = command_dict.get("action")
    # command.subject = command_dict.get("subject")
    # command.value = float(command_dict.get("value")) if check_float(command_dict.get("value")) else 0.

    # command.unit = command_dict.get("unit")

@app.post("/mfccs")
async def receive_mfccs(mfccs: List[List[float]]):

    if not ready_to_receive:
        return Response(False, "robot not ready to receive a new command").body
    """
    Endpoint to receive MFCCs data from a mobile app.
    """

    mfccs_array = np.array(mfccs)

    #!###################################################################
    # TODO: Pass MFCCs to the Speech Reconition Pipeline
    #!###################################################################

    print("all good")

    return Response(True, "data passed to model").body    

# ! audio and video routes
# !############################################################
@app.post('/audio')
async def process_audio(audio: UploadFile = File(...)):

    if not ready_to_receive:
        return Response(False, "robot not ready to receive a new command").body
    """
    Endpoint to receive an audio file and extract MFCCs.
    """
    audio_bytes = await audio.read()

    with tempfile.NamedTemporaryFile(delete=False, suffix='.wav') as temp_file:
        temp_filename = temp_file.name
        temp_file.write(audio_bytes)

    
    audio, sr = librosa.load(temp_filename, sr=None)

    
    number_mfccs = 13


    output_folder = "./API/Data/MFCCs/mfccs.npy"


    mfccs = extract_mfcc(audio, sr, number_mfccs, output_folder)

    #!###################################################################
    # TODO: Pass MFCCs to the Speech Reconition Pipeline
    #!###################################################################

    os.unlink(temp_filename)
    return Response(True, "data passed to model").body

@app.post('/data/video')
async def receive_video(frames: UploadFile = File(...)):

    if not ready_to_receive:
        return Response(False, "robot not ready to receive a new command").body

    """
    Endpoint to receive video data from a mobile app.
    """


    video_bytes = await frames.read()

    with tempfile.NamedTemporaryFile(delete=False) as temp_file:
        temp_file.write(video_bytes)
        temp_filename = temp_file.name


    cap = cv2.VideoCapture(temp_filename)

    command = "" #* will be the output of the model

    #!###################################################################
    # TODO: Pass Video to the Sign Reconition Pipeline, the returned text should be stored in command
    #!###################################################################

    #! this is just a simulation of the model
    #?#########################################
    cmd_decoder =CommandsDecoder()


    dummy_cmds = cmd_decoder.dummy_pipline

    random_cmd=np.random.choice(dummy_cmds)

    print(random_cmd)
    #?#########################################

    # command_tuple = cmd_decoder.label_command(random_cmd)

    # TODO:  Replace random_cmd with command
    decoded_cmd = decoder.decode_cmd(random_cmd)

    communicator.publish_user_command(decoded_cmd)

    cap.release()
    os.unlink(temp_filename)

    return Response(True, "decoded command", random_cmd).body

# !############################################################


    
uvicorn_run(app, host="127.0.0.1", port=8000)

