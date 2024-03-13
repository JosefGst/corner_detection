#!/usr/bin/env python
import rospy
from laser_line_extraction.msg import LineSegmentList, LineSegment
from std_msgs.msg import String

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "debug string: %s", data.header.frame_id)
    line = LineSegmentList
    line_entry = LineSegment

    
    rospy.loginfo(rospy.get_caller_id() + "debug string: %d", data.line_segments.size())


    
def listener():
    rospy.init_node('corner_detection', anonymous=True)

    rospy.Subscriber("line_segments", LineSegmentList, callback)

    rospy.spin()

if __name__ == '__main__':
    listener()