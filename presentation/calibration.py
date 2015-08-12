import rospy
from std_msgs.msg import String


class MAPSInterface(object):
    def __init__(self):

        self.publisher = rospy.Publisher('/MAPScommand', String, queue_size=1)
        #self.publisher.publish('autocalibrate %f' % (0.0))

    def calibrate(self, percent):
        for i in range(5):
            self.publisher.publish('autocalibrate %f' % (percent))
            rospy.Rate(1000).sleep()

#def calibrate(percent):
#    interface = MAPSInterface()
#    interface.calibrate(percent/100.0)
