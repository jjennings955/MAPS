import rospy
import aggregation

def set_threshhold(which, threshold):
    """
    Set the calibration threshhold of a sensor
    :param which: The sensor ID to calibrate
    :param threshold: The threshhold to calibrate to
    :return: None
    """
    rospy.set_param('calibration_threshold/%d' % which, threshold)

def get_all_thresholds():
    """
    Get all calibration threshholds that have been set
    :return: A dictionary mapping sensor ID to threshhold
    """
    return rospy.get_param('calibration_threshold')

def get_threshold(which):
    """
    Get the calibration threshold of a specific sensor
    :param which: The sensor to retrieve calibration information for
    :return: The calibration threshhold of the specified sensor
    """
    return rospy.get_param('calibration_threshold/%d' % (which))

def auto_calibrate(percent_increase=0.1):
    """
    Automatically calibrate the system using the current values as a baseline, and the specified percent increase
    :param percent_increase: The percent increase above the current value to calibrate to
    :return: None
    """
    current_values = aggregation.get_current_values()
    for k,v in current_values.iteritems():
        set_threshhold(int(k), (1+percent_increase)*v)
