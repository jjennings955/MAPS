import rospy
import aggregation

def set_threshholds(threshhold_dict):
    """Set the calibration threshholds. T should contain an ordered list mapping sensor ID to calibration threshhold in PSI"""
    rospy.set_param('calibration_threshold', threshhold_dict)

def set_threshhold(which, threshhold):
    rospy.set_param('calibration_threshold/%d' % which, threshhold)

def get_all_thresholds(t):
    return rospy.get_param('calibration_threshold')

def get_threshhold(which):
    return rospy.get_param('calibration_threshold/%d' % (which))

def auto_calibrate(percent_increase=0.1):
    current_values = aggregation.get_current_values()
    for k,v in current_values.iteritems():
        set_threshhold(int(k), (1+percent_increase)*v)
