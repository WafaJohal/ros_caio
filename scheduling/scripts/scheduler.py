#!/usr/bin/env python
"""
CAIO scheduler
-recieve the plans from the planning module
- call the action manager for the execution
"""
import rospy
from std_msgs.msg import String



def enChaine(checks):
	return "{ P : "+str(checks["P"])+", "+"D : "+str(checks["D"])+", "+"E : "+str(checks["S"])+"}"

def perform_action_client(action):
    rospy.wait_for_service('action_player')
    try:
        perform_action = rospy.ServiceProxy('action_player', ActionPlayer)
        resultat = perform_action(action)
        return resultat
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e


def callback(data):
    # gets the stimuli utterences
    plans = data.data
    # declares the prolog engine
    rospy.loginfo("new plans received")
    prolog = json_prolog.Prolog()
    ##TODO : QUERY and res
    # DO query for the next action
    action="coucou"
    #call action service and get callback when done
    rospy.loginfo("action performed "+perform_action_client(action))
    rospy.loginfo(data.data +"out : scheduling")
  
    
def schedule():

    # in ROS, nodes are unique named. 
    rospy.init_node('scheduler', anonymous=True)

	# subscribe to utterance stimulus of type string
	# it should be prolog formatted
    sub = rospy.Subscriber('plans', String, callback)
 

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
    
    
    


if __name__ == '__main__':
    try:
        schedule()
    except rospy.ROSInterruptException: pass
