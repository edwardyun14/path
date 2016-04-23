
import sys

# Import ROS files
import rospy
#from std_msgs.msg import Int16MultiArray
from nav_msgs.msg import Path
from geometry_msgs.msg import Pose, Point, PoseStamped
from std_msgs.msg import Header
# Import the path generator files to enable communication
import map_to_path


# Send JSON file information to path generator
# Alan

# Receive coordinates (array of tuples) from path generator

#creates a standard header with the stamp field set
def create_std_h():
    h=Header()
    h.stamp = rospy.Time.now()
    return h


# Publish coordinates to ROS
def publisher(paths):
    publisher = rospy.Publisher('pathfinder', Path, queue_size=20)
    rospy.init_node('path_publisher', anonymous=True)
    rate = rospy.Rate(10) # 10 hertz
    msgs=[] #list for all the messages we need to publish
    for path in paths:
        msg = Path() #new path message
        msg.header = create_std_h() #add a header to the path message
        for x,y,z in path:
            #create a pose for each position in the path
            #position is the only field we care about, leave the rest as 0
            ps = PoseStamped()
            ps.header=create_std_h()
            i=Pose()
            i.position=Point(x,y,z)
            ps.pose=i
            msg.poses.append(ps)
        
        msgs.append(msg)
    while not rospy.is_shutdown(): # while we're going
        #publish each message
        for m in msgs:
            #rospy.loginfo(m)
            publisher.publish(m)
        rate.sleep()

# Main
if __name__ == "__main__":
    if len(sys.argv)<2:
        print "error, requires command line argument of map file name"
    else:
        path = map_to_path.createPathFromFile(sys.argv[1])
        #path=path[0] #TODO: fix this, temporary fix to remove ambiguity
        try:
            publisher(path)
        except rospy.ROSInterruptException:
            pass
