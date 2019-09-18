#!/usr/bin/env python

import rospkg
import numpy as np
from sklearn import preprocessing
from sklearn.model_selection import train_test_split
from sklearn import svm
from sklearn.svm import SVC
from sklearn.metrics import classification_report
from sklearn.metrics import accuracy_score, precision_score, recall_score
from sklearn.metrics import confusion_matrix
from sklearn.metrics import f1_score
from sklearn.model_selection import validation_curve
from sklearn.model_selection import learning_curve
import pickle

import matplotlib.pyplot as plt
import seaborn as sns

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
print(labels.ravel())
print(labels)
#features = preprocessing.minmax_scale(data[:, 4:])
features = data[:, 4:]
#print(features)
x_train, x_test, y_train, y_test = train_test_split(features, labels.ravel(), test_size=test_size)

clf = svm.SVC(kernel=svm_kernel, C=svm_C, gamma=svm_gamma)
clf.fit(x_train, y_train)
predict=clf.predict(x_test)
print('accuracy_score')
print(accuracy_score(y_test, predict))
print('')

print(classification_report(y_test, predict))

#print('confusion matrix')
#confusion_matrix = confusion_matrix(y_test, predict,labels=[0.0,1.0,2.0,3.0,4.0])
#print(confusion_matrix)
#sns.heatmap(confusion_matrix,annot=True)
#plt.show()

train_sizes, train_scores, test_scores = learning_curve(clf,x_train, y_train, cv=10,train_sizes=[0.1,0.2,0.3,0.4,0.5,0.6, 0.7, 0.8, 0.9, 1.0],n_jobs=-1)

train_scores_mean = np.mean(train_scores, axis=1)
train_scores_std  = np.std(train_scores, axis=1)
test_scores_mean = np.mean(test_scores, axis=1)
test_scores_std  = np.std(test_scores, axis=1)

plt.grid()

plt.xlabel('#training samples')
plt.ylabel('accuracy')
plt.legend(loc='lower right')
plt.ylim([0.4, 1.01])

plt.fill_between(train_sizes, train_scores_mean - train_scores_std,train_scores_mean + train_scores_std, alpha=0.1,color="r")
plt.plot(train_sizes, train_scores_mean, 'o-', color="r",label="Training score")
plt.plot(train_sizes, test_scores_mean, 'o-', color="g",label="Cross-validation score")
plt.legend(loc="best")

plt.show()

model_filepath = package_path + '/model/'
model_filename = 'model.sav'
pickle.dump(clf, open(model_filepath + model_filename, 'wb'))