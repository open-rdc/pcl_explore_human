#!/usr/bin/env python

import rospkg
import numpy as np
from sklearn import preprocessing
from sklearn import svm
import pickle
import yaml

class svm_predict:
    def __init__(self):
        self.rospack = rospkg.RosPack()
        self.package_path=self.rospack.get_path('pcl_explore_human')
        self.f = open(self.package_path+'/config/'+'svm_predict.yaml', "r+")
        self.f_param = yaml.load(self.f)
        self.is_predict=self.f_param["is_predict"]
        self.model_filepath = self.package_path + '/model/'
        self.model_filename = 'model.sav'
        self.filepath = self.model_filepath + self.model_filename
        self.clf=pickle.load(open(self.filepath,'rb'))
    
    def predict(self,data):
        predict=self.clf.predict(np.array([data]))
        return predict

if __name__ == "__main__":
    dl=svm_predict()
    