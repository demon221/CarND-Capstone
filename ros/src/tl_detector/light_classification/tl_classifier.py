import cv2
import numpy as np
import tensorflow as tf
import h5py
import os
from keras.backend.tensorflow_backend import set_session
from keras.models import Sequential
from keras.layers import Convolution2D, Flatten, Dense, MaxPooling2D, Dropout
from keras.utils.np_utils import to_categorical
from keras import losses, optimizers, regularizers
from keras.models import load_model
from styx_msgs.msg import TrafficLight

cwd = os.path.dirname(os.path.realpath(__file__))

class TLClassifier(object):
    def __init__(self):
        #TODO load classifier
        os.chdir(cwd)
        self.model = load_model('tl_classify_sim.h5')
        
        # This trick makes Keras happy - Thanks to Eric Lavigne for the tip
        self.graph = tf.get_default_graph()
        
    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        #TODO implement light color prediction
        light_image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB);
        resized_image = cv2.resize(light_image, (32,64))
        processed_image = resized_image/255.
        
        # Keras trick part 2 - Thanks again Eric
        with self.graph.as_default():
            y_hat = self.model.predict(processed_image.reshape((1, 64, 32, 3)))
            enum_color = y_hat[0].tolist().index(np.max(y_hat[0]))

            return enum_color




