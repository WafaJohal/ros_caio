#!/usr/bin/env python
#import roslib; roslib.load_manifest('json_prolog')
import rospy
#import json_prolog
from std_msgs.msg import String

pub = rospy.Publisher('checks', String, queue_size=10)


def enChaine(checks):
	return "{ N : "+str(checks["N"])+", "+"P : "+str(checks["P"])+", "+"G : "+str(checks["G"])+", "+"O : "+str(checks["O"])+", "+"I : "+str(checks["I"])+"}"
	

def callback(data):
    # gets the stimuli utterences
    mental_attitude = data.data
    # declares the prolog engine
    rospy.loginfo("new data recived")
    #prolog = json_prolog.Prolog()
    checks={"N":0.0,"P":0.0,"G":0.0,"O":0.0,"I":0.0}
    #query = prolog.query("scherer_checks(N,P,G,O,I)")
    #for solution  in query.solutions() :
	#	checks["N"] = float(solution["N"])
	#	checks["P"] = float(solution["P"])
	#	checks["G"] = float(solution["G"])
	#	checks["O"] = float(solution["O"])
	#	checks["I"] = float(solution["I"])
    
    # sends a request for bdi attitudes computed from  the utterance
    ###query = prolog.query("stimulus(Agt,"+utterance+")")
	### example de queryrosrun
    #query = prolog.query("member(A, [1, 2, 3, 4]), B = ['x', A]")
    # gets the first solution
   
    #for solution in query.solutions():
    #    print 'Found solution. A = %s, B = %s' % (solution['A'], solution['B'])
        
    #first_solution = query.solutions()
    #print 'Found solution. A = %s, B = %s' % (first_solution['A'], first_solution['B'])
    
    # closes the querry bridge
    #query.finish()
    
    # print info
    new_check = data.data #TODO CHANGE MSGS FORMAT
    rospy.loginfo(new_check)
    #publish the info in the bdi_attitude topic
    #pub.publish(str(data)+ " :  recieved "+new_check)
    pub.publish("out : appraisal_secs ")
def appraisal_secs():

    # in ROS, nodes are unique named. 
    rospy.init_node('appraisal_secs', anonymous=True)

	# subscribe to utterance stimulus of type string
	# it should be prolog formatted
    sub = rospy.Subscriber('bdi_attitude', String, callback)
    
    # publishes a bdi attitude of type string
	# it is prolog formatted
    pub = rospy.Publisher('checks', String, queue_size=10)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
    
    
    


if __name__ == '__main__':
    try:
        appraisal_secs()
    except rospy.ROSInterruptException: pass
