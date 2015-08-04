import rospy

def set_windowsize(n=10):
    """Maintain an internal history of n sensor readings"""
    rospy.set_param('aggregation_windowsize', n)
def get_current_values():
    return rospy.get_param('current_value', {})
def get_current_value(which):
    return rospy.get_param('current_value/%d' % which)

def initialize(n):
    for i in range(n):
        rospy.set_param('current_value/%d' % i, 0)

def update(which, val):
    rospy.set_param('current_value/%d' % which, val)
#history = rospy.get_param('/sensor_history', [])
