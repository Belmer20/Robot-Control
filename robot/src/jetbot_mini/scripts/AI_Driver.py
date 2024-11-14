import numpy as np
import cv2
import mediapipe as mp
import os
from jetbot_mini.msg import Commands_msg
from ai_utils import create_lstm_model, recognize_sign
import tensorflow as tf
from PIL import Image, ImageDraw, ImageFont
import matplotlib.pyplot as plt


class AI_Driver:
    __instance = None

    def __new__(cls, *args, **kwargs):
        if cls.__instance is None:
            cls.__instance = super(AI_Driver, cls).__new__(cls)
            cls.__instance.__initialized = False
        return cls.__instance

    def __init__(self):
        if not self.__initialized:
            self.name = "ai driver"
            self.signs_actions = actions = np.array(['forward', 'backward', 'stop', 'left', 'right'])
            self.mode = 0

            # ! Paths
            self.models_path = os.path.join('/'.join(os.path.abspath(__file__).split('/')[:-1]), 'AI/models')
            self.sign_recognition_model_path = os.path.join(self.models_path, 'SignRecognition')
            self.Speech_recognition_model_path = os.path.join(self.models_path, 'SpeechRecognition')
            self.Obstacle_detection_model_path = os.path.join(self.models_path, 'ObstacleDetection')

            # ! Load the Models
            self.sign_recognition_model = self.load_Sign_recognition_model()
            self.speech_recognition_model = self.load_Speech_recognition_model()
            self.obstacle_detection_model = self.load_Obstacle_Detection_model()

            self.obstacle_detection_threshold = 0.6

            self.__initialized = True

    def switch_mode(self):
        self.mode = not self.mode

        if self.mode:
            self.del_signs_model()
            self.load_Speech_recognition_model()
        else:
            self.del_speech_model()
            self.load_Sign_recognition_model()

    #! the following function only load/delete models to/from memory
    #!##############################################################################################################################################################
    def load_Sign_recognition_model(self):
        model = None
        for model_name in os.listdir(self.sign_recognition_model_path):
            if 'active' in model_name:
                model_weights_path = os.path.join(self.sign_recognition_model_path, model_name)

                # this is not tested, IDK if this is the right way so check it
                model = create_lstm_model()
                model.load_weights(model_weights_path)
                return model
        return None

    def load_Speech_recognition_model(self):
        model = None
        for model_name in os.listdir(self.Speech_recognition_model_path):
            if 'active' in model_name:
                #######################################################################
                #TODO: Load model here
                #######################################################################

                return model
        return None

    def load_Obstacle_Detection_model(self):
        model = None
        for model_name in os.listdir(self.Obstacle_detection_model_path):
            if 'active' in model_name:
                #######################################################################
                # TODO: Load model here
                #######################################################################
                model_weights_path = os.path.join(self.Obstacle_detection_model_path, model_name)
                
                # Load the TFLite model and allocate tensors
                model = tf.lite.Interpreter(model_path= model_weights_path)
                model.allocate_tensors()

                return model
            
        return None

    # !##############################################################################################################################################################

    def del_signs_model(self):
        if hasattr(self, "sign_recognition_model"):
            del self.sign_recognition_model

    def del_speech_model(self):
        if hasattr(self, "speech_recognition_model"):
            del self.speech_recognition_model

    def del_obstacle_model(self):
        if hasattr(self, "obstacle_detection_model"):
            del self.obstacle_detection_model

    # ! the following functions are to use the models
    # !##############################################################################################################################################################
    def speech_recognize(self, signal):
        """
            ! INPUTS: 
                ? audio signal
            ! OUTPUTS:
                ? extracted text from the signal
        """
        if hasattr(self, "speech_recognition_model"):
            # TODO
            pass

    def sign_recognize(self, frames):
        """
            ! INPUTS: 
                ? sequence of frames
            ! OUTPUTS:
                ? extracted text from the gesture
        """

        if hasattr(self, "sign_recognition_model"):
            # TODO: make sure this works
            recognize_sign(self.sign_recognition_model, frames, self.actions)
            pass

        
        # Function to draw bounding boxes on the image
    # def draw_boxes(image, boxes, scores, threshold=0.55):
    #     draw = ImageDraw.Draw(image)
    #     font = ImageFont.truetype("C:/Windows/Fonts/Arial.ttf", 12)  # Replace with your desired font and size
    #     for i in range(len(boxes)):
    #         if scores[i] >= threshold:
    #             ymin, xmin, ymax, xmax = boxes[i]
    #             (left, right, top, bottom) = (xmin * image.width, xmax * image.width,
    #                                         ymin * image.height, ymax * image.height)
    #             # Draw bounding box
    #             draw.rectangle([(left, top), (right, bottom)], outline='red', width=line_width)
    
    
    
    def obstacle_detection(self, frames):
        """
            ! INPUTS:
                ? sequence of frames from the main camera of the main robot
            ! OUTPUTS:
                ? boxing of the model
                ? [optional] recognize the object (by name)
        """

        frame = frames[0]
        if frame is None:
            return False
        

        if hasattr(self, "obstacle_detection_model"):
            
            # TODO
            pass

        # ! IMPORTANT:
        # ! the structure of the returned dictionary is
        # {"top_left_coords": 50, "top_right_coords": 250,"bottom_right_coords": 250, "bottom_left_coords": 50 }

        
            # Define line width for bounding boxes
            line_width = 2

            # Load frame
            image = Image.fromarray(frame).convert('RGB')  # Convert to RGB to handle PNG with transparency
            model=self.obstacle_detection_model
            input_details=self.obstacle_detection_model.get_input_details()
            output_details=self.obstacle_detection_model.get_output_details()

            # Resize the image to the model's required input size
            input_shape = input_details[0]['shape']
            image_resized = image.resize((input_shape[1], input_shape[2]))


            # Convert the image to a numpy array and ensure it is of type UINT8
            input_data = np.array(image_resized, dtype=np.uint8)

            # Ensure the tensor matches the expected shape by the model
            input_data = np.expand_dims(input_data, axis=0)  # Add batch dimension if required

            # Set the tensor to point to the input data to be inferred
            model.set_tensor(input_details[0]['index'], input_data)

            # Run the inference
            model.invoke()

            # Get the results
            boxes = model.get_tensor(output_details[0]['index'])[0]  # Bounding box coordinates of detected objects
            classes = model.get_tensor(output_details[1]['index'])[0]  # Class index of detected objects
            scores = model.get_tensor(output_details[2]['index'])[0]  # Confidence of detected objects
            num_detections = int(model.get_tensor(output_details[3]['index'])[0])  # Total number of detected objects
            
            def transform_coordinates(boxes):
                transformed_boxes = []
                
                for index, box in enumerate(boxes):
                    # rand1, rand2, rand3, rand4 = box
                    if scores[index] > self.obstacle_detection_threshold:
                        ymin, xmin, ymax, xmax = box

                        (left, right, top, bottom) = (xmin * image.width, xmax * image.width,
                                                ymin * image.height, ymax * image.height)
                        
                        

                        transformed_box = {
                            "left": left,
                            "right":  right,
                            "top": top,
                            "bottom": bottom
                        }

                        transformed_boxes.append(transformed_box)

                return transformed_boxes
            

            transformed_boxes = transform_coordinates(boxes)

       

        # top_left, top_right, bottom_right, bottom_left = np.random.choice(coords)

        return  transformed_boxes


# !##############################################################################################################################################################
test = AI_Driver()

import cv2

# Read the image in color mode (default)
image = cv2.imread('/home/alpha/Desktop/Project-2CS/robot/src/jetbot_mini/scripts/test.jpeg')

# Check if image is read successfully
if image is None:
  print("Error: Could not read image!")
else:
  boxes = test.obstacle_detection([image])
  print(boxes[:5])
  # You can now process the image
  # ... (your image processing code)

#   # Display the image (optional)
#   cv2.imshow('Image', image)
#   cv2.waitKey(0)  # Wait for a key press to close the window
#   cv2.destroyAllWindows()


# test.obstacle_detection([])