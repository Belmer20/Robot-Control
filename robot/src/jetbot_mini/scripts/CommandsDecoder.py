import numpy as np
from utils import check_float
import json

conf_file_path = "./conf.json"

class CommandsDecoder:
    def __init__(self):
        

            
        
        

        self.dummy_pipline = ["make the shape", "make a straight line", "move in the shape of C","move in the shape of H","turn right 32.75 d","turn right 71.5 degrees","make left 87.25 degree","turn right 36.75 degree","move in the shape of B","draw S","make a turn left 29.25 radian","turn left 70.0 rad","make a turn left 73.5 radian","turn right 10.5 radians","make a turn right 95.0 deg"]

        self.actions = ['move', 'turn', 'advance', 'make a turn', 'make', 'make a straight line', 'move in a straight line', 'in a', 'keep moving', 'draw', 'write', 'move in the shape of', 'move in the shape', 'write the letter', 'move in', 'in the', 'the shape', 'shape of', 'write the', 'the letter', 'make a', 'a turn', "a straight", "straight line", 'make a letter', 'a letter', 'make an', 'make the shape of', 'make the', 'make a', 'draw a', 'make']

        self.subjects = ['forward', 'backward', 'left', 'right', 'circle', 'circle with radius', 'right angle', 'left angle']
        with open(conf_file_path) as conf_file:
            json_file = json.load(conf_file)
            alphabet_json_en = json_file['Alphabet_en']
            alphabet_json_ar = json_file['Alphabet_ar']
            shapes_json = json_file['Shapes']
        self.subjects = np.concatenate([self.subjects, list(alphabet_json_ar.keys()), list(alphabet_json_en.keys()), list(shapes_json.keys())]) 


        # self.subjects = np.concatenate([self.subjects, alphabet_json_en.keys(), alphabet_json_ar.keys(), shapes_json.keys()])

        self.units = ['m', 'meter', 'meters','s', 'second', 'seconds', 'm', 'minute', 'minutes','d', 'deg', 'degree', 'degrees', 'r', 'rad', 'radian', 'radians']

        self.vocab = np.concatenate([self.units, self.actions, self.subjects])
        

        self.action_classes = {"straight": ['move', 'advance', 'make a straight line', 'move in a straight line','keep moving'],
                               "turns": ['turn', 'make a turn'],
                               "shapes": ['draw', 'draw a', 'draw an', 'make', 'make an' 'write', 'move in the shape of', 'write the letter', 'make a', 'make an', 'make the shape', 'move in the shape']}

        self.subject_classes = {"shapes": np.concatenate([list(alphabet_json_ar.keys()), list(alphabet_json_en.keys()), list(shapes_json.keys())]),
                                "directions":['forward', 'backward', 'left', 'right'],
                                "angles": ['right angle', 'left angle']}

        self.unit_classes = {"distance": ['m', 'meter', 'meters'],
                             "time": ['s', 'second', 'seconds', 'm', 'minute', 'minutes'],
                             "angle": ['d', 'deg', 'degree', 'degrees', 'r', 'rad', 'radian', 'radians']
                             }
        
        self.value_classes = {}
        self.class_symbols = {"<A>": self.action_classes, "<S>": self.subject_classes, "<V>": self.value_classes, "<U>": self.unit_classes}

    def identify_class(self, token, dict):
        for key in dict.keys():
            if token in dict.get(key):
                return key
        return ""

    def label_command(self, command):
        labels = {'action': "", 'subject': "",'value': "",'unit': ""}
        labels_classes = {'action': "", 'subject': "",'value': "",'unit': ""}
        cmd_tokens = command.split(" ")
        cmd_tokens.append("<END>")
        i = 0
        while i < (len(cmd_tokens)-1) and cmd_tokens[i] != "<END>":
            if check_float(cmd_tokens[i]):
                labels["value"] = float(cmd_tokens[i])
                labels_classes["value"] = "float"
            elif cmd_tokens[i] in self.actions:
                cont = True
                j=i
                action = cmd_tokens[j]
                while cont and cmd_tokens[j+1] != "<END>":
                    concat_action = " ".join([cmd_tokens[j], cmd_tokens[j+1]])
                    if concat_action in self.actions:
                        action += " " + cmd_tokens[j+1]
                        j+=1
                    else:
                        cont = False
                if not cont or cmd_tokens[j+1] == "<END>":
                    i = j
                labels["action"] = action
                labels_classes["action"] = self.identify_class(action, self.action_classes)
            elif cmd_tokens[i] in self.subjects:
                labels["subject"] = cmd_tokens[i]
                labels_classes["subject"] = self.identify_class(cmd_tokens[i], self.subject_classes)
            elif cmd_tokens[i] in self.units:
                labels["unit"] = cmd_tokens[i]
                labels_classes["unit"] = self.identify_class(cmd_tokens[i], self.unit_classes)
            i+=1
        return labels, labels_classes
