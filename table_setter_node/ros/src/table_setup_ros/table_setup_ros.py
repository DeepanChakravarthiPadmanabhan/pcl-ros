import tf
import rospy
import std_msgs
from geometry_msgs.msg import PolygonStamped

class table_setup_ros:
    def __init__(self):
        '''Subscriber'''
        self.convex_hull_subscriber_ = rospy.Subscriber('convex_hull_input', PolygonStamped, self.table_display_cb)
        self.table_setup_event_in_ = rospy.Subscriber("set_table_trigger", std_msgs.msg.String, self.event_in_cb)

    def table_display_cb(self, polygon_convex_hull):
        rospy.loginfo("Displaying table task")

    def event_in_cb(self,msg):
        if msg.data == 'set_table':
            rospy.loginfo("Received trigger to display table")



def main():
    rospy.init_node('[table setter node] started')
    object_ = table_setup_ros()
    object_.start()

