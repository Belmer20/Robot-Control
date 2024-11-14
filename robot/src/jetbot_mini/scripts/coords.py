
import os
import sys
import json
import numpy as np

current_dir = '/'.join( os.path.abspath(sys.argv[0]).split('/')[:-1] )
os.chdir(current_dir)
conf_file_path = "./conf.json"

class Alphabet:

    def __init__(self):
        self.alphabet_en = self.init_alpha_en()
        self.alphabet_ar = self.init_alpha_ar()
       

        

    def init_alpha_en(self):
        with open(conf_file_path) as conf_file:
            alphabet_json = json.load(conf_file)['Alphabet_en']
            return alphabet_json
        
    def init_alpha_ar(self):
        with open(conf_file_path) as conf_file:
            alphabet_json = json.load(conf_file)['Alphabet_ar']
            return alphabet_json
        

class Shapes:

    def __init__(self):
        self.shapes = self.init_shapes()

    def init_shapes(self):
        with open(conf_file_path) as conf_file:
            shapes_json = json.load(conf_file)['Shapes']
            return shapes_json


class Coords:
    def __init__(self):
        self.all_coords = [Alphabet().alphabet_en, Alphabet().alphabet_ar]
        self.all_shapes = [Shapes().shapes]

