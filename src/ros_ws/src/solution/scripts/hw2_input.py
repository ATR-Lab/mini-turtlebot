#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import Int16
from std_msgs.msg import String

name_pub = rospy.Publisher('name_topic', String, queue_size=10)
operation_pub = rospy.Publisher('operation_topic', String, queue_size=10)
num1_pub = rospy.Publisher('num1_topic', Int16, queue_size=10)
num2_pub = rospy.Publisher('num2_topic', Int16, queue_size=10)

def result_callback(data):
    result = data.data
    rospy.loginfo("the result I am getting is: " + str(result))
    userinput()

def main():
    rospy.init_node('talker', anonymous=True)
    rospy.Subscriber("result_topic", Int16, result_callback)

    userinput()

    rospy.spin()

def userinput():
    name = raw_input("What is your name? ")
    operation = raw_input("What operation do you want to perform? ")
    num1 = input("Enter the first number: ")
    num2 = input("Enter the second number: ")

    name_pub.publish(name)
    operation_pub.publish(operation)
    num1_pub.publish(num1)
    num2_pub.publish(num2)
    
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass