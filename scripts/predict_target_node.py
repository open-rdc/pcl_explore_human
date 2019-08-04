#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Float32
from geometry_msgs.msg import Point
from geometry_msgs.msg import PointStamped

from svm_predict import *

class Publisher_target_predict():
    def __init__(self):
        self.publisher = rospy.Publisher('/predict_target_label',Float32, queue_size=1) 
        self.publisher2 = rospy.Publisher('/target_point',PointStamped, queue_size=1) 
        self.label = Float32()
        self.target_point =PointStamped()
    
    def make_msg(self,label,description):
        self.label.data=label
        self.target_point.header.stamp=rospy.Time.now()
        self.target_point.header.frame_id='odom'
        self.target_point.point.x=np.array(description.data)[0]
        self.target_point.point.y=np.array(description.data)[1]
        self.target_point.point.z=np.array(description.data)[2]


    def send_msg(self):
        self.publisher.publish(self.label)
        self.publisher2.publish(self.target_point) 

class Subscriber_target_predict():
    def __init__(self,pub):
        self.sub = rospy.Subscriber("description",Float32MultiArray, self.callback)

        self.description=Float32MultiArray()

        self.ml=svm_predict()
    
    def callback(self,description):
        if self.ml.is_predict:
            pub.make_msg(self.ml.predict(description.data),description)
            pub.send_msg()

if __name__  == '__main__':

    rospy.init_node('predict_target')
    pub=Publisher_target_predict()
    sub=Subscriber_target_predict(pub)
    
    try:
        rospy.spin()
    except rospy.ROSInterruptException: pass