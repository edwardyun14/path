
# Import ROS files
import rospy
from std_msgs.msg import Int16MultiArray

# Import the path generator files to enable communication
from path import *
import tkinter as tk

# Send JSON file information to path generator
# Alan

# Receive coordinates (array of tuples) from path generator


# Publish coordinates to ROS
def publisher(path):
    publisher = rospy.Publisher('pathfinder', Int16MultiArray, queue_size=20)
    rospy.init_node('path_publisher', anonymous=True)
    rate = rospy.Rate(10) # 10 hertz
    while not rospy.is_shutdown():
        # Pass each tuple in the patha rray
        for j,k in path:
            rospy.loginfo(j,k)
            publisher.publish(j,k)
        rate.sleep()

# Main
if __name__ == "__main__":

    path = find_path()

    try:
        publisher(path)
    except rospy.ROSInterruptException:
        pass
