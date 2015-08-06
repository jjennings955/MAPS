import rospy
from std_msgs.msg import String


class MAPSInterface(object):
    def __init__(self):
       # rospy.init_node('MAPS_interface', anonymous=True)
        self.publisher = rospy.Publisher('/MAPScommand', String, queue_size=1)

    def calibrate(self, percent):
        self.publisher.publish('autocalibrate %f' % (percent))
        rospy.Rate(1000).sleep()

def calibrate(percent):
    interface = MAPSInterface()
    interface.calibrate(percent/100.0)
