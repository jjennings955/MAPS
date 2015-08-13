import rospy
from std_msgs.msg import String

class MAPSInterface(object):
    def __init__(self):
        self.publisher = rospy.Publisher('/MAPScommand', String, queue_size=1)

    def calibrate(self, percent):
        print("Calibrating")
        self.publisher.publish('autocalibrate %f' % (percent))
