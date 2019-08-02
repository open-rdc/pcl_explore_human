#!/usr/bin/env python

import rospy
import rospkg
import numpy as np
from sklearn import preprocessing
from sklearn.model_selection import train_test_split
from sklearn import svm

rospack = rospkg.RosPack()
filepath=rospack.get_path('pcl_explore_human')+'/dataset/'+'/description.csv'

data=np.loadtxt( filepath ,delimiter=',',dtype=float);
labels = data[:, 0:1]
features = preprocessing.minmax_scale(data[:, 1:])
x_train, x_test, y_train, y_test = train_test_split(features, labels.ravel(), test_size=0.3)

clf = svm.SVC(kernel='rbf', C=1, gamma='auto')
