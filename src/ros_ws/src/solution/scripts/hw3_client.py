#!/usr/bin/env python
import rospy
from std_msgs.msg import Time
from datetime import datetime
import time
from solution.srv import *

#init publisher at gloabl to use it at anywhere in the code
timePub = rospy.Publisher('local_time', Time, queue_size=10)

def local_time_client(location):
    #ping and wait for server to be up and running
    rospy.wait_for_service('worldTime')

    try:
        # init connection
        calTime = rospy.ServiceProxy('worldTime', timeserver)
        # send request with input
        resultTime = calTime(location)
        #get result from server and return
        return resultTime.remoteTime
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e


def main():
    # init node
    rospy.init_node('localtimer', anonymous=True)
    rate = rospy.Rate(10) # 10hz

    #keep program from running, keep looping
    while not rospy.is_shutdown():
        #get python input
        rospy.loginfo("Enter an Location for Calculate World Time: ")
        loc = raw_input()

        # get system time and convert to ros Time formate and publish
        loc_time_sec = rospy.Time.from_sec(time.time())
        timePub.publish(loc_time_sec)

        #send request to server for timer update
        rospy.loginfo("the time at " + loc + " is: " 
                    + str(datetime.fromtimestamp(local_time_client(loc).to_sec())))

        rate.sleep()

if __name__ == '__main__':
    main()