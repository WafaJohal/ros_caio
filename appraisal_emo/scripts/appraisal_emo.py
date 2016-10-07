#!/usr/bin/env python
import roslib; roslib.load_manifest('json_prolog')
import rospy
import json_prolog
from std_msgs.msg import String

pub = rospy.Publisher('emotions', String, queue_size=10)


def enChaine(checks):
	return "{ P : "+str(checks["P"])+", "+"D : "+str(checks["D"])+", "+"E : "+str(checks["E"])+", "+"T : "+str(checks["T"])+"}"

def updateTime():
	prolog = json_prolog.Prolog()
	q1 = prolog.query("instant(T)")
	solution = q1.nextsolution()
	result = str(solution)
	return result #to finish, global variable set time etc


def callback(data):
    # gets the stimuli utterences
    mental_attitude = data.data
    # declares the prolog engine
    rospy.loginfo("new data recived")
    prolog = json_prolog.Prolog()
    sols={"P":0.0,"D":0.0,"E":0.0,"T":0}
    emotion=""
    # sends a request for emotion
    #query = prolog.query("j_emotion(agent, E,P,D,T"))
	
    #for solution in query.solutions():
    #	emotion = solution["E"]   
    #	sols["P"] = solution["P"]
	#	sols["D"] = solution["D"]
	#	sols["E"] = solution["E"]
    
    
    # closes the querry bridge
    #query.finish()
    
    # print info
    new_emo = enChaine(sols) #TODO CHANGE MSGS FORMAT
    rospy.loginfo(new_emo)
    #publish the info in the bdi_attitude topic
    #pub.publish(str(data)+ " :  recieved "+new_check)
    pub.publish("out : appraisal_emo ")
    
def appraisal_emo():

    # in ROS, nodes are unique named. 
    rospy.init_node('appraisal_emo', anonymous=True)

	# subscribe to utterance stimulus of type string
	# it should be prolog formatted
    sub = rospy.Subscriber('bdi_attitude', String, callback)
    
    # publishes a bdi attitude of type string
	# it is prolog formatted
    pub = rospy.Publisher('emotions', String, queue_size=10)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
    
    
    


if __name__ == '__main__':
    try:
        appraisal_emo()
    except rospy.ROSInterruptException: pass
