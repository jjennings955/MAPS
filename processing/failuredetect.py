import rospy
import aggregation

DEFAULT_FLOOR = 0
DEFAULT_CEIL = 500

def set_fail_threshhold(which, threshhold_floor, threshhold_ceil):
    """
    Set upper and lower bounds on a sensor's reasonable values
    :param which: The sensor id to set failure thresholds for
    :param threshhold_floor: The lower bound below which a failure is triggered
    :param threshhold_ceil: The upper bound above which a failure is triggered
    :return: None
    """
    rospy.set_param('floor_threshold/%d' % which, threshhold_floor)
    rospy.set_param('ceil_threshold/%d' % which, threshhold_ceil)

def get_floor_threshold(which):
    """
    Get the lower bound for specified sensor
    :param which: The sensor to retrieve lower bound for
    :return: The lowest reasonable value for specified sensor (in PSI)
    """
    return rospy.get_param('floor_threshold/%d' % which, 0)

def get_ceil_threshold(which):
    """
    Get the upper bound for specified sensor
    :param which: The sensor to retrieve upper bound for
    :return: The highest reasonable value for specified sensor (in PSI)
    """
    return rospy.get_param('ceil_threshold/%d' % which, 500)

def get_floor_thresholds():
    """
    Get the lower bound for all sensors, in form of a dictionary
    :return: Dictionary mapping sensor ID to threshholds
    """
    return rospy.get_param('floor_threshold')

def get_ceil_thresholds():
    """
    Get the upper bound for all sensors, in form of a dictionary
    :return: Dictionary mapping sensor ID to threshholds
    """
    return rospy.get_param('ceil_threshold')

def auto_fail(floor_percent=0.1, ceil_percent=2.0):
    """
    Automatically set failure threshholds using a percent decrease/increase over current value
    :param floor_percent: The percent decrease to set as the floor
    :param ceil_percent: The percent increase to set as the ceiling
    :return: None
    """
    current_values = aggregation.get_current_values()
    for k,v in current_values.iteritems():
        set_fail_threshhold(int(k), floor_percent*v, ceil_percent*v)

def auto_fail_absolute(floor_val=12.0, ceil_val=20.0):
    """
    Automatically set failure threshholds using absolute values
    :param floor_val: The value for the lower bound
    :param ceil_val: The value for the upper bound
    :return: None
    """
    current_values = aggregation.get_current_values()
    for k,v in current_values.iteritems():
        set_fail_threshhold(int(k), floor_val, ceil_val)
