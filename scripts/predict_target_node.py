#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import PointStamped

from svm_predict import *

class Subscribe_target_predict:
    def __init__(self):
        self.sub = rospy.Subscriber("description",Float32MultiArray, self.callback)

        self.message=Float32MultiArray()

        self.ml=svm_predict()
    
    def callback(self,message):
        self.predict=self.ml.predict(message.data)

class Publisher_target_predict:
    

if __name__  == '__main__':
    rospy.init_node('predict_target')
    sub=Subscribe_target_predict()
    
    try:
        rospy.spin()
    except rospy.ROSInterruptException: pass