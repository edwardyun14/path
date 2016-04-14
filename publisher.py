
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

def create_std_h():
    h=Header()
    h.stamp = rospy.Time.now()
    return h


# Publish coordinates to ROS
def publisher(path):
    publisher = rospy.Publisher('pathfinder', Path, queue_size=20)
    rospy.init_node('path_publisher', anonymous=True)
    rate = rospy.Rate(10) # 10 hertz
    msg = Path()
    #msg.poses = [Pose().position=Point(x, y, z) for x,y,z in path]
    msg.header = create_std_h()
    for x,y,z in path:
        ps = PoseStamped()
        ps.header=create_std_h()
        i=Pose()
        i.position=Point(x,y,z)
        ps.pose=i
        msg.poses.append(ps)
    
    while not rospy.is_shutdown():
        # Pass each tuple in the patha rray
        
        
##        for p in path:
##            msg = Path()
##            msg.pose.position = p
        rospy.loginfo(msg)
        publisher.publish(msg)
        rate.sleep()

# Main
if __name__ == "__main__":
    if len(sys.argv)<2:
        print "error, requires command line argument of map file name"
    else:
        path = map_to_path.createPathFromFile(sys.argv[1])
        path=path[0] #TODO: fix this, temporary fix to remove ambiguity
        try:
            publisher(path)
        except rospy.ROSInterruptException:
            pass
