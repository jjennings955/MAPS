import rospy

def set_filter(state):
    rospy.set_param('filtering', state)
def get_filter_mode(state):
    return rospy.get_param('filtering', 0)
def set_filter_params(params):
    rospy.set_param('filter_weights', params)
def apply_filter(state):
