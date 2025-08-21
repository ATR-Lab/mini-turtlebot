#!/usr/bin/env python

from beginner_tutorials.srv import AddTwoInts,AddTwoIntsResponse
from std_msgs.msg import String
import rospy

def handle_add_two_ints(req):
    print "Returning [%s + %s = %s]"%(req.a, req.b, (req.a + req.b))
    return AddTwoIntsResponse(req.a + req.b)

def chatter_callback(data):
    rospy.loginfo(data + 5)

def add_two_ints_server():
    rospy.init_node('add_two_ints')
    rospy.Subscriber("chatter", String, chatter_callback)
    s = rospy.Service('add_two_ints', AddTwoInts, handle_add_two_ints)
    print "Ready to add two ints."
    rospy.spin()

if __name__ == "__main__":
    add_two_ints_server()