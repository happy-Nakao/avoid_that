#!/usr/bin/env python3
#-*- coding:utf-8 -*-

import rospy
import smach
from enter_room.srv import EnterRoom
from happymimi_navigation.srv import NaviLocation



class Enter(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes = ['enter_finish'])
        self.enter_room = rospy.ServiceProxy('/enter_room_server', EnterRoom)
        
    def execute(self):
        self.enter_room(1.0, 0.5)
        return 'enter_finish'

class Navi(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes = ['navi_finish','all_finish'],
                             input_keys = ['p_num_in'])
        self.navi_srv = rospy.ServiceProxy('navi_location_server', NaviLocation)
        
    def execute(self, userdata):
        p_num = userdata.p_num_in
        print(p_num)
        if p_num == 0:
            self.navi_srv('living')
        elif p_num == 1:
            self.navi_srv('long_table')
        elif p_num == 2:
            self.navi_srv('white_table')
        else:
            return 'all_finish'
        
        return 'navi_finish'
    
class Return(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes = ['return_finish'],
                             input_keys = ['p_num_in'],
                             output_keys = ['p_num_out'])
        self.navi_srv = rospy.ServiceProxy('navi_location_server', NaviLocation)
        
    def execute(self, userdata):
        count = userdata.p_num_in
        count = count + 1
        userdata.p_num_out = count
        self.navi_srv('entrance')
        return 'return_finish'

if __name__ == '__main__':
    rospy.init_node('avoid_that_master')
    rospy.loginfo("Start Avoid That")
    sm = smach.StateMachine(outcomes = ['success'])
    sm.userdata.p_num = 0
    
    with sm:
        smach.StateMachine.add("Enter",
                               Enter(),
                               transitions = {"enter_finish":"Navi"})
        smach.StateMachine.add("Navi",
                               Navi(),
                               transitions = {"navi_finish":"Return",
                                              "all_finish":"success"},
                               remapping = {"p_num_in":"p_num"})
        smach.StateMachine.add("Return",
                               Return(),
                               transitions = {"return_finish":"Navi"},
                               remapping = {"p_num_in":"p_num",
                                            "p_num_out":"p_num"})
                               
    outcome = sm.execute()
    
    