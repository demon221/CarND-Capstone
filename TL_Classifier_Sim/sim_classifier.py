# Trafic Light Classificator
import cv2
import glob
import matplotlib.pyplot as plt
import numpy as np
import tensorflow as tf
from keras.backend.tensorflow_backend import set_session
from keras.models import Sequential
from keras.layers import Convolution2D, Flatten, Dense, MaxPooling2D, Dropout
from keras.utils.np_utils import to_categorical
from keras import losses, optimizers, regularizers
from keras.models import load_model 

X_train = []
x_label = []
for img_class, directory in enumerate(['red', 'yellow', 'green', 'none']):
    for i, file_name in enumerate(glob.glob("{}/*.jpg".format(directory))):
        file = cv2.imread(file_name)

        file = cv2.cvtColor(file, cv2.COLOR_BGR2RGB);
        resized = cv2.resize(file, (32,64))

        X_train.append(resized/255.)
        x_label.append(img_class)
        
X_train = np.array(X_train)
x_label = np.array(x_label)

categorical_labels = to_categorical(x_label)

model = Sequential()
model.add(Convolution2D(16,3,3,input_shape=(64,32,3),subsample=(2,2),activation="relu"))
model.add(Convolution2D(32,3,3,subsample=(2,2),activation="relu"))
model.add(Flatten())
model.add(Dropout(0.2))  #dropout layer to reduce overfitting
model.add(Dense(8))
model.add(Dropout(0.1))
model.add(Dense(4, activation='softmax'))
model.compile(loss="mse", optimizer='adam', metrics=['accuracy'])  #using adam optimizer to fit a regression model

model.fit(X_train, categorical_labels, batch_size=32, epochs=300, verbose=True, validation_split=0.1, shuffle=True)

print(model.evaluate(X_train, categorical_labels, verbose=0))
import h5py
model.save('tl_classify_sim.h5')

# test model"

#just test it using training sets
model = load_model('tl_classify_sim.h5')

prediction = model.predict(X_train)
for i in range(len(prediction)):
    print (i, categorical_labels[i])

