import cv2
import numpy as np
import tensorflow as tf
import h5py
from keras.backend.tensorflow_backend import set_session
from keras.models import Sequential
from keras.layers import Convolution2D, Flatten, Dense, MaxPooling2D, Dropout
from keras.utils.np_utils import to_categorical
from keras import losses, optimizers, regularizers
from keras.models import load_model
from styx_msgs.msg import TrafficLight

class TLClassifier(object):
    def __init__(self):
        #TODO load classifier
        print "********************* Loading classifier *********************"
        yifeng_classifier = load_model('/home/student/CarND-Capstone/ros/src/tl_detector/light_classification/TL_SIM.h5')

    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        #TODO implement light color prediction
        print "   Processing image . . ."
        light_image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB);
        resized_image = cv2.resize(light_image, (32,64))
        processed_image = resized_image/255.
        
        prediction = yifeng_classifier.predict(processed_image)
        print "     Prediction: ", prediction

        return TrafficLight.UNKNOWN
