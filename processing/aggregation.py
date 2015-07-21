import rospy

def set_windowsize(n=10):
    """Maintain an internal history of n sensor readings"""
    rospy.set_param('aggregation_windowsize', n)