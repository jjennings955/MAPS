import rospy

def set_threshholds(threshhold_dict):
    """Set the calibration threshholds. T should contain an ordered list mapping sensor ID to calibration threshhold in PSI"""
    rospy.set_param('calibration_threshold', threshhold_dict)

def set_threshhold(which, threshhold):
    rospy.set_param('calibration_threshold/%d' % which, threshhold)

def get_all_thresholds(t):
    return rospy.get_param('calibration_threshold')

def get_threshhold(which):
    return rospy.get_param('calibration_threshold/%d' % (which))