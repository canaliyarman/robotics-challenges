import rospy
import tf2_ros
import numpy as np
from move_base_msgs.msg import Costmap  
from geometry_msgs.msg import PoseStamped
from nav_msgs.srv import GetMap, GetMapResponse
def callback(data):
    print(data)
if __name__ == '__main__':
    rospy.init_node('pose_listener_python')
    rospy.Subscriber('/move_base/global_costmap/costmap', Costmap, callback, queue_size=1)
    rospy.spin()
