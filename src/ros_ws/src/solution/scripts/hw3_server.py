#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from std_msgs.msg import Time
from datetime import datetime
import time
from solution.srv import timeserver, timeserverResponse

local_time = Time()     #init a variable with "ROS Time" type

def hourToSec(hour):
    return hour*60*60

def handleTimeCal(req):
    global local_time
    # there may be a case that user send time request even before "local_time" topic receive data 
    if local_time != None:
        remote = Time()
        # list of choise
        if req.location == "Japan":
            # time differece here is 11 hour
            remote = local_time - rospy.Duration(hourToSec(11))
        elif req.location == "Australia":
            # time differece here is 10 hour
            remote = local_time - rospy.Duration(hourToSec(10))
        
        rospy.loginfo("There is a request coming from Kent at " + str(datetime.fromtimestamp(local_time.to_sec())))
        #return result back to client
        return timeserverResponse(remote)



def clientTimeCallback(data):
    #give value to global variable for later use
    global local_time
    local_time = data.data
    
def main():
    #init node
    rospy.init_node('worldTimeServer', anonymous=True)
    #init subscriber
    rospy.Subscriber("local_time", Time, clientTimeCallback)
    #init server
    worldServer = rospy.Service('worldTime', timeserver, handleTimeCal)

    rospy.spin()

if __name__ == '__main__':
    main()