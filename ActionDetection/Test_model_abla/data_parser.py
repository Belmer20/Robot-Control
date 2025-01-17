import os
import csv
import pdb 
from collections import namedtuple

ListDataJpeg = namedtuple('ListDataJpeg', ['id', 'label', 'path'])
ListDataGulp = namedtuple('ListDataGulp', ['id', 'label'])

class JpegDataset(object):

    def __init__(self, csv_path_input, csv_path_labels, data_root):
        self.csv_data = self.read_csv_input(csv_path_input, data_root)
        self.classes = self.read_csv_labels(csv_path_labels)
        self.classes_dict = self.get_two_way_dict(self.classes)
    
    def read_csv_input(self, csv_path, data_root):
                csv_data = []
                with open(csv_path) as csvfile:
                    csv_reader = csv.reader(csvfile, delimiter=';')
                    for row in csv_reader:
                      
                        item = ListDataJpeg(row[-1], row[-2], os.path.join(data_root, row[0]))
                        csv_data.append(item)
                return csv_data
    
    def read_csv_labels(self, csv_path):
        classes = []
        with open(csv_path) as csvfile:
            csv_reader = csv.reader(csvfile)
            for row in csv_reader:
                classes.append(row[0])
        return classes

    def get_two_way_dict(self, classes):
        classes_dict = {}
        for i, item in enumerate(classes):
            classes_dict[item] = i
            classes_dict[i] = item
        return classes_dict

class GulpDataset(object):

    def __init__(self, csv_path_input, csv_path_labels):
        self.csv_data = self.read_csv_input(csv_path_input)
        self.classes = self.read_csv_labels(csv_path_labels)
        self.classes_dict = self.get_two_way_dict(self.classes)

    def read_csv_input(self, csv_path):
        csv_data = []
        with open(csv_path) as csvfile:
            csv_reader = csv.reader(csvfile, delimiter=';')
            for row in csv_reader:
                item = ListDataGulp(row[0], row[1])
                csv_data.append(item)
        return csv_data

    def read_csv_labels(self, csv_path):
        classes = []
        with open(csv_path) as csvfile:
            csv_reader = csv.reader(csvfile)
            for row in csv_reader:
                classes.append(row[0])
        return classes

    def get_two_way_dict(self, classes):
        classes_dict = {}
        for i, item in enumerate(classes):
            classes_dict[item] = i
            classes_dict[i] = item
        return classes_dict
