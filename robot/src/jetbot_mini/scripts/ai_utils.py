#!/usr/bin/python3

import os
os.environ['TF_CPP_MIN_LOG_LEVEL'] = '2'
from tensorflow.keras.models import Sequential
from tensorflow.keras.layers import LSTM, Dense
from tensorflow.keras.callbacks import TensorBoard
from tensorflow.keras.layers import Dropout
from tensorflow.keras.callbacks import EarlyStopping
from tensorflow.keras.layers import Input
from scikeras.wrappers import KerasClassifier
from keras.optimizers import Adam
os.environ['TF_CPP_MIN_LOG_LEVEL'] = '0'

import cv2
import numpy as np
import mediapipe as mp

def create_lstm_model(units_lstm1=64, units_lstm2=256,units_lstm3=200, dropout_rate=0.1, units_dense1=150, units_dense2=32):
    actions = np.array([ 'forward', 'backward', 'stop', 'left', 'right'])

    model = Sequential()
    model.add(Input(shape=(30, 1662)))
    model.add(LSTM(units_lstm1, return_sequences=True, activation='relu'))
    model.add(LSTM(units_lstm2, return_sequences=True, activation='relu'))
    model.add(Dropout(dropout_rate))
    model.add(LSTM(units_lstm3, return_sequences=True, activation='relu'))
    model.add(Dropout(dropout_rate))
    model.add(LSTM(units_lstm2, return_sequences=False, activation='relu'))
    model.add(Dropout(dropout_rate))
    model.add(Dense(units_dense1, activation='relu'))
    model.add(Dropout(dropout_rate))
    model.add(Dense(units_dense2, activation='relu'))
    model.add(Dense(actions.shape[0], activation='softmax'))
    optimizer = Adam(learning_rate=0.0005)
    model.compile(optimizer=optimizer, loss='categorical_crossentropy', metrics=['categorical_accuracy'])
    # early_stopping = EarlyStopping(monitor='val_categorical_accuracy', patience=13, restore_best_weights=True)
    return model



def mediapipe_detection(image, model):
    image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB) # COLOR CONVERSION BGR 2 RGB
    image.flags.writeable = False                  # Image is no longer writeable
    results = model.process(image)                 # Make prediction
    image.flags.writeable = True                   # Image is now writeable 
    image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR) # COLOR COVERSION RGB 2 BGR
    return image, results


def extract_keypoints(results):
    pose = np.array([[res.x, res.y, res.z, res.visibility] for res in results.pose_landmarks.landmark]).flatten() if results.pose_landmarks else np.zeros(33*4)
    face = np.array([[res.x, res.y, res.z] for res in results.face_landmarks.landmark]).flatten() if results.face_landmarks else np.zeros(468*3)
    lh = np.array([[res.x, res.y, res.z] for res in results.left_hand_landmarks.landmark]).flatten() if results.left_hand_landmarks else np.zeros(21*3)
    rh = np.array([[res.x, res.y, res.z] for res in results.right_hand_landmarks.landmark]).flatten() if results.right_hand_landmarks else np.zeros(21*3)
    return np.concatenate([pose, face, lh, rh])

colors = [(245,117,16), (117,245,16), (16,117,245),(117,245,16), (16,117,245)]
def prob_viz(res, actions, input_frame, colors):
    output_frame = input_frame.copy()
    for num, prob in enumerate(res):
        cv2.rectangle(output_frame, (0,60+num*40), (int(prob*100), 90+num*40), colors[num], -1)
        cv2.putText(output_frame, actions[num], (0, 85+num*40), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,255,255), 2, cv2.LINE_AA)
        
    return output_frame


def recognize_sign(model, cap, actions):
    sequence = []
    sentence = []
    mp_holistic = mp.solutions.holistic # Holistic model
    mp_drawing = mp.solutions.drawing_utils # Drawing utilities
    threshold = 0.9
    with mp_holistic.Holistic(min_detection_confidence=0.5, min_tracking_confidence=0.5) as holistic:
        # Read feed
        ret, frame = cap.read()
        # Make detections
        image, results = mediapipe_detection(frame, holistic)
        # print(results)
        
        # Draw landmarks
        # draw_styled_landmarks(image, results)
            
        # 2. Prediction logic
        keypoints = extract_keypoints(results)
    #     sequence.insert(0,keypoints)
    #     sequence = sequence[:30]
        sequence.append(keypoints)
        sequence = sequence[-30:]
            
        if len(sequence) == 30:
            res = model.predict(np.expand_dims(sequence, axis=0))[0]
            print(actions[np.argmax(res)])
                
                
        #3. Viz logic
            if res[np.argmax(res)] > threshold: 
                if len(sentence) > 0: 
                    if actions[np.argmax(res)] != sentence[-1]:
                        sentence.append(actions[np.argmax(res)])
                else:
                    sentence.append(actions[np.argmax(res)])

            if len(sentence) > 5: 
                sentence = sentence[-5:]

            # Viz probabilities
            image = prob_viz(res, actions, image, colors)
    return sentence
        # cv2.rectangle(image, (0,0), (640, 40), (245, 117, 16), -1)
        # cv2.putText(image, ' '.join(sentence), (3,30), 
        #             cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2, cv2.LINE_AA)
            
        #     # Show to screen
        # cv2.imshow('OpenCV Feed', image)