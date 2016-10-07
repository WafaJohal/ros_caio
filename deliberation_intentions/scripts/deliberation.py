#!/usr/bin/env python
#import roslib; roslib.load_manifest('json_prolog')
import rospy
#import json_prolog
from std_msgs.msg import String

pub = rospy.Publisher('intentions', String, queue_size=10)


def enChaine(checks):
	return "{ P : "+str(checks["P"])+", "+"D : "+str(checks["D"])+", "+"E : "+str(checks["S"])+"}"



def callback(data):
    # gets the stimuli utterences
    mental_attitude = data.data
    # declares the prolog engine
    rospy.loginfo("new data received")
    #prolog = json_prolog.Prolog()
    sols={"P":0.0,"D":0.0,"S":0.0}
    #T = callForTime()
    # load full KB
    
    emotion=""
    # sends a request for emotion
    #query = prolog.query("intend(agent, P,D,T,S"))
	
    #for solution in query.solutions():   
    #	sols["P"] = solution["P"]
	#	sols["D"] = solution["D"]
	#	sols["S"] = solution["S"]	
    
    # closes the querry bridge
    #query.finish()
    
    # print info
    new_emo = data.data #TODO CHANGE MSGS FORMAT
    rospy.loginfo(new_emo)
    #publish the info in the bdi_attitude topic
    #pub.publish(str(data)+ " :  recieved "+new_check)
    pub.publish("out : deliberation")
    
def deliberation():

    # in ROS, nodes are unique named. 
    rospy.init_node('deliberation', anonymous=True)

	# subscribe to utterance stimulus of type string
	# it should be prolog formatted
    sub = rospy.Subscriber('bdi_attitude', String, callback)
    
    # publishes a bdi attitude of type string
	# it is prolog formatted
    pub = rospy.Publisher('intentions', String, queue_size=10)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
    
    
    


if __name__ == '__main__':
    try:
        deliberation()
    except rospy.ROSInterruptException: pass
