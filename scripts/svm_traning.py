#!/usr/bin/env python

import rospy
import rospkg
import numpy as np
from sklearn import preprocessing
from sklearn.model_selection import train_test_split
from sklearn import svm
from sklearn.metrics import accuracy_score, precision_score, recall_score
import pickle
import matplotlib.pyplot as plt

import yaml

rospack = rospkg.RosPack()
package_path=rospack.get_path('pcl_explore_human')

f = open(package_path+'/config/'+'/pcl_extraction_human_description.yaml', "r+")
f_param = yaml.load(f)

filename=f_param["description_filename"]

filepath=package_path+'/dataset/'+filename

data=np.loadtxt( filepath ,delimiter=',',dtype=float)
labels = data[:, 0:1]
print(labels)
#features = preprocessing.minmax_scale(data[:, 1:])
features = data[:, 1:]
print(features)
x_train, x_test, y_train, y_test = train_test_split(features, labels.ravel(), test_size=0.3)

clf = svm.SVC(kernel='rbf', C=10, gamma='auto')
clf.fit(x_train, y_train)
predict=clf.predict(x_test)
print(accuracy_score(y_test, predict), precision_score(y_test, predict), recall_score(y_test, predict))

model_filepath = package_path + '/model/'
model_filename = 'model.sav'
pickle.dump(clf, open(model_filepath + model_filename, 'wb'))

