__author__ = 'jason'
import rospy
import aggregation

DEFAULT_FLOOR = 0
DEFAULT_CEIL = 500

def set_threshholds(threshhold_dict):
    """Set the calibration threshholds. T should contain an ordered list mapping sensor ID to calibration threshhold in PSI"""
    rospy.set_param('floor_threshold', threshhold_dict)
    rospy.set_param('ceil_threshold', threshhold_dict)

def set_threshhold(which, threshhold_floor, threshhold_ceil):
    rospy.set_param('floor_threshold/%d' % which, threshhold_floor)
    rospy.set_param('ceil_threshold/%d' % which, threshhold_ceil)

def get_floor_threshold(which):
    return rospy.get_param('floor_threshold/%d' % which, 0)

def get_ceil_threshold(which):
    return rospy.get_param('ceil_threshold/%d' % which, 500)

def get_floor_thresholds(t):
    return rospy.get_param('floor_threshold')

def get_ceil_thresholds(t):
    return rospy.get_param('ceil_threshold')

def auto_fail(floor_percent=0.1, ceil_percent=2.0):
    current_values = aggregation.get_current_values()
    for k,v in current_values.iteritems():
        set_threshhold(int(k), floor_percent*v, ceil_percent*v)
