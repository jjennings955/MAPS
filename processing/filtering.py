import rospy
import numpy as np

FILTER_NONE = 0
FILTER_FIR = 1
FILTER_MEDIAN = 2

def filtering_initialize():
    set_filter(FILTER_NONE)

def set_filter(state):
    rospy.set_param('filtering', state)

def get_filter_mode():
    return rospy.get_param('filtering', 0)

def set_filter_params(params):
    rospy.set_param('filter_weights', params)

def get_filter_weights():
    rospy.get_param('filter_weights')

def apply_filter(values):
    filter_mode = get_filter_mode()
    if filter_mode == FILTER_FIR:
        weights = get_filter_weights()
        val = np.dot(values, weights)
        return val[0]
    if filter_mode == FILTER_MEDIAN:
        val = np.median(values)
        return val
