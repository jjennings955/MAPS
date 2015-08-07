import rospy
#from rospy import *
from std_msgs.msg import String
from calibration import set_threshholds, get_threshhold, auto_calibrate
#import random
from filtering import set_filter
from communication import readSerial
import aggregation

def debug(foo):
    print foo

class EventManager(object):
    def __init__(self):
        rospy.init_node('MAPS_talker', anonymous=True)
        self.publisher = rospy.Publisher('pressure_events', String, queue_size=1)
        self.command_manager = rospy.Subscriber("MAPScommand", String, self.handle_event)
        self.failure_publisher = rospy.Publisher('failure_events', String, queue_size=1)
    def handle_event(self, data):
        command, arg = data.data.split(' ')

        if command == 'autocalibrate':
            auto_calibrate(float(arg))
        elif command == 'enable_fir':
            set_filter(1)
        elif command == 'enable_median':
            set_filter(2)
        elif command == 'disable_filter':
            set_filter(0)

    def publish_packet(self, sensor_id, sensor_value):
        threshhold = get_threshhold(sensor_id)
        if sensor_value > threshhold:
            self.publisher.publish("Sensor: %d, Pressure: %f" % (sensor_id, sensor_value))

    def check_failures(self, sensor_id, sensor_value, fail_ceil, fail_floor):
        if sensor_value > fail_ceil or sensor_value < fail_floor:
            self.failure_publisher.publish("Sensor: %d" % (sensor_id))
            return True
        return False

    def sleep(self):
        rospy.Rate(1000).sleep()

if __name__ == '__main__':
    try:
        set_threshholds(dict(zip([str(i) for i in range(16)], [0 for i in range(16)])))
        eventManager = EventManager()
        while True:
            val, pin_id = readSerial()
            eventManager.publish_packet(pin_id, val)
            aggregation.update(pin_id, val)
            eventManager.sleep()
    except rospy.ROSInterruptException:
        pass