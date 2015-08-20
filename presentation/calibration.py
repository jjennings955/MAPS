import rospy
from std_msgs.msg import String

class MAPSInterface(object):
    def __init__(self):
        """
        Create a MAPSInterface object for sending commands to the ROS node
        :return: None
        """
        self.publisher = rospy.Publisher('/MAPScommand', String, queue_size=1)

    def calibrate(self, percent):
        """
        Send a calibration command
        :param percent: The percent increase above the current values to calibrate to
        :return: None
        """
        print("Calibrating")
        self.publisher.publish('autocalibrate %f' % (percent))
