import rospy
import numpy as np

FILTER_NONE = 0
FILTER_FIR = 1
FILTER_MEDIAN = 2

def filtering_initialize():
    """
    Initialize the filtering system to the default value (no filtering)
    :return: None
    """
    set_filter(FILTER_NONE)

def set_filter(state):
    """
    Change the filtering mode
    :param state: The mode to change to
    :return: None
    """
    rospy.set_param('filtering', state)

def get_filter_mode():
    """
    Retrieve the current filtering mode for the ROS Node
    :return: FILTER_NONE, FILTER_FIR, or FILTER_MEDIAN depending on the current mode
    """
    return rospy.get_param('filtering', 0)

def set_filter_params(params):
    """
    Set the filter weights for FIR filter
    :param params: A list of weights
    :return: None
    """
    rospy.set_param('filter_weights', params)

def get_filter_weights():
    """
    Get the filter weights for the FIR filter
    :return: A list of weights
    """
    rospy.get_param('filter_weights')

def apply_filter(values):
    """
    Apply the current filter to set of values and return the result
    :param values: The values to run through filter
    :return: The filtered value
    """
    filter_mode = get_filter_mode()
    if filter_mode == FILTER_FIR:
        weights = get_filter_weights()
        val = np.dot(values, weights)
        return val[0]
    if filter_mode == FILTER_MEDIAN:
        val = np.median(values)
        return val
