#!/usr/bin/env python
#import roslib; roslib.load_manifest('json_prolog')

import rospy
#import json_prolog
from std_msgs.msg import String

pub = rospy.Publisher('bdi_attitude', String, queue_size=10)

def callback(data):
    # gets the stimuli utterences
    utterance = data.data
    # declares the prolog engine
    #prolog = json_prolog.Prolog()
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
    new_bdi = utterance
    rospy.loginfo(new_bdi)
    #publish the info in the bdi_attitude topic
    #pub.publish(new_bdi)
    pub.publish("out perception ")
    
def perception_bdi():

    # in ROS, nodes are unique named. 
    rospy.init_node('perception_bdi', anonymous=True)

	# subscribe to utterance stimulus of type string
	# it should be prolog formatted
    sub = rospy.Subscriber("stimuli_utterance", String, callback)
    
    # publishes a bdi attitude of type string
	# it is prolog formatted
    pub = rospy.Publisher('bdi_attitude', String, queue_size=10)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
    
    
    


if __name__ == '__main__':
    try:
        perception_bdi()
    except rospy.ROSInterruptException: pass
