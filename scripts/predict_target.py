#!/usr/bin/env python

import rospy
import rospkg
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import PointStamped

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

def callback(description):
    data=np.array(description.data)
    #print(data)
    #features = np.array([preprocessing.minmax_scale(data)])
    features = np.array([data])
    #print(features)

    predict=loaded_clf.predict(features)
    if(predict[0]==1.0){

    }

def target_predict():
    rospy.init_node('target_predict')
    rospy.Subscriber('description',Float32MultiArray,callback)
    rospy.spin()

if __name__  == '__main__':
    rospack = rospkg.RosPack()
    package_path=rospack.get_path('pcl_explore_human')
    model_filepath = package_path + '/model/'
    model_filename = 'model.sav'
    filepath = model_filepath + model_filename
    print(filepath)
    loaded_clf=pickle.load(open(filepath,'rb'))

    target_predict()