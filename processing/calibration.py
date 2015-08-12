import rospy
import aggregation

def set_threshhold(which, threshhold):
    rospy.set_param('calibration_threshold/%d' % which, threshhold)

def get_all_thresholds(t):
    return rospy.get_param('calibration_threshold')

def get_threshold(which):
    return rospy.get_param('calibration_threshold/%d' % (which))

def auto_calibrate(percent_increase=0.1):
    current_values = aggregation.get_current_values()
    for k,v in current_values.iteritems():
        set_threshhold(int(k), (1+percent_increase)*v)
