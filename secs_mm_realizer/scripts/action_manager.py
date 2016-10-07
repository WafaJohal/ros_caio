#!/usr/bin/env python

import rospy
from secs_mm_realizer.srv import *
from std_msgs.msg import String





def handle_action(req):
	#todo send request to perform action to robot handler
    rospy.loginfo(" return the after action is performed")
    return ActionPlayerResponse(req.stra + "Wafa")

  
    
def action_manager():
    # in ROS, nodes are unique named. 
    rospy.init_node('action_manager', anonymous=True)
    s = rospy.Service('action_player', ActionPlayer, handle_action)
    rospy.loginfo("Ready to play action")
   
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
    
    
    


if __name__ == '__main__':
    try:
        action_manager()
    except rospy.ROSInterruptException: pass
