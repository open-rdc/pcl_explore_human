#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Int8

from svm_predict import *

class Publisher_target_predict():
    def __init__(self):
        self.publisher = rospy.Publisher('/predict_target_label',Int8, queue_size=10) 
        self.message= Int8()
    
    def send_msg(self):
        self.publisher.publish(self.message)

class Subscriber_target_predict():
    def __init__(self,pub):
        self.sub = rospy.Subscriber("description",Float32MultiArray, self.callback)

        self.message=Float32MultiArray()

        self.ml=svm_predict()
    
    def callback(self,message):
        self.predict=self.ml.predict(message.data)
        #print(self.predict)
        pub.message.data=self.predict
        pub.send_msg()

if __name__  == '__main__':

    rospy.init_node('predict_target')
    pub=Publisher_target_predict()
    sub=Subscriber_target_predict(pub)
    
    try:
        rospy.spin()
    except rospy.ROSInterruptException: pass