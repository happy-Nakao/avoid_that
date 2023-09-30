#!/usr/bin/env python3
#-*- coding:utf-8 -*-

import rospy
import smach
from enter_room.srv import EnterRoom
from happymimi_navigation.srv import NaviLocation
from grasping_object.srv import RecognitionToGrasping
#from dynamixel_controller import StrTrg
from happymimi_recognition_msg.srv import RcognitionList
from std_msgs.msg import Float64
from happymimi_msgs.srv import StrTrg

class Enter(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes = ['enter_finish'])
        self.enter_room = rospy.ServiceProxy('/enter_room_server', EnterRoom)
        
    def execute(self):
        self.enter_room(1.0, 0.5)
        return 'enter_finish'

class Navi(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes = ['navi_finish'])
        self.navi_srv = rospy.ServiceProxy('navi_location_server', NaviLocation)
        
    def execute(self):
        self.navi_srv('living')
        return 'navi_finish'
    
class Grasp(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes = ['grasp_finish'])
        self.grasp_srv = rospy.ServiceProxy('/recognition_to_grasping', RecognitionToGrasping)
        self.recog_srv = rospy.ServiceProxy('/recognition/list' , RecognitionList)
        self.head_pub = rospy.Publisher('/servo/head', Float64, queue_size = 1 )
        self.navi_srv = rospy.ServiceProxy('navi_location_server', NaviLocation)

        
    def execute(self, userdata):
        self.navi_srv('table')
        #grasp_name = userdata.object_name_in
        rospy.sleep(2.0)
        rospy.loginfo('pick')
        self.head_pub.publish(25.0)
        rospy.sleep(2.0)
        self.grasp_srv(target_name = 'any')
        return 'grasp_finish'
        
class Give(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes = ['give_finish'])
        self.navi_srv = rospy.ServiceProxy('navi_location_server', NaviLocation)
        self.arm_srv = rospy.ServiceProxy('/servo/arm', StrTrg)
        
    def execute(self):
        self.navi_srv('operator')
        self.arm_srv('give')
        return 'grasp_success'
    
if __name__ == '__main__':
    rospy.init_node('avoid_that_arc')
    rospy.loginfo("Start Avoid That")
    sm = smach.StateMachine(outcomes = ['success'])
    
    with sm:
        smach.StateMachine.add("Enter",
                               Enter(),
                               transitions = {"enter_finish":"Navi"})
        smach.StateMachine.add("Navi",
                               Navi(),
                               transitions = {"navi_finish":"Grasp"})
        smach.StateMachine.add("Grasp",
                               Grasp(),
                               transitions = {"grasp_finish":"Give"})
        smach.StateMachine.add("Give",
                               Give(),
                               transitions = {"give_finish":"success"})
                               
    outcome = sm.execute()

