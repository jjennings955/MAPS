import rospy
#from rospy import *
from std_msgs.msg import String
from calibration import set_threshholds, get_threshhold
import random

class EventManager(object):
    def __init__(self):
        self.publisher = rospy.Publisher('pressure_events', String, queue_size=100)
        rospy.init_node('MAPS_talker', anonymous=True)
    def publish_packet(self, sensor_id, sensor_value):
        threshhold = get_threshhold(sensor_id)
        if sensor_value > threshhold:
            self.publisher.publish("Sensor: %d, Pressure: %f" % (sensor_id, sensor_value))
    def sleep(self):
        pass
        #rospy.Rate(1000).sleep()

if __name__ == '__main__':
    try:
        set_threshholds(dict(zip([str(i) for i in range(10)], [26 for i in range(10)])))
        eventManager = EventManager()
        while True:
            eventManager.publish_packet(random.randint(0,9), random.randint(0,36))
            eventManager.sleep()
    except rospy.ROSInterruptException:
        pass