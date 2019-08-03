#!/usr/bin/env python

import rospkg
import numpy as np
from sklearn import preprocessing
from sklearn.model_selection import train_test_split
from sklearn import svm
from sklearn.metrics import accuracy_score, precision_score, recall_score
import pickle

import yaml

rospack = rospkg.RosPack()
package_path=rospack.get_path('pcl_explore_human')

f = open(package_path+'/config/'+'/svm_training.yaml', "r+")
f_param = yaml.load(f)

filename=f_param["description_filename"]
delimiter=f_param["data_delimiter"]
test_size=f_param["test_size"]
svm_kernel=f_param["svm_kernel"]
svm_C=f_param["svm_C"]
svm_gamma=f_param["svm_gamma"]
filepath=package_path+'/dataset/'+filename


data=np.loadtxt( filepath ,delimiter=delimiter,dtype=float)
labels = data[:, 0:1]
#print(labels)
features = preprocessing.minmax_scale(data[:, 1:])
#features = data[:, 1:]
#print(features)
x_train, x_test, y_train, y_test = train_test_split(features, labels.ravel(), test_size=test_size)

clf = svm.SVC(kernel=svm_kernel, C=svm_C, gamma=svm_gamma)
clf.fit(x_train, y_train)
predict=clf.predict(x_test)
#print('accuracy_score=',accuracy_score(y_test, predict), 'precision_score=',precision_score(y_test, predict), 'recall_score=',recall_score(y_test, predict))
print('accuracy_score')
print(accuracy_score(y_test, predict))
print('precision_score')
print(precision_score(y_test, predict))
print('recall_score')
print(recall_score(y_test, predict))

model_filepath = package_path + '/model/'
model_filename = 'model.sav'
pickle.dump(clf, open(model_filepath + model_filename, 'wb'))

