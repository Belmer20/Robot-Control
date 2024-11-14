
import os
import sys
import json
import numpy as np

current_dir = '/'.join( os.path.abspath(sys.argv[0]).split('/')[:-1] )
os.chdir(current_dir)
conf_file_path = "./conf.json"


class Alphabet:

    def __init__(self):
        self.alphabet = self.init_alpha()

    def init_alpha(self):
        with open(conf_file_path) as conf_file:
            alphabet_json = json.load(conf_file)['Alphabet']
            return alphabet_json
