import rospy
from std_msgs.msg import String
from calibration import  get_threshold, auto_calibrate
from filtering import set_filter
import json

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

    def check_threshold(self, sensor_id, sensor_value):
        threshold = get_threshold(sensor_id)
        return sensor_value > threshold

    def publish_packet(self, sensor_id, sensor_value, debug=False):
        if debug or self.check_threshold(sensor_id, sensor_value):
            self.publisher.publish(EventManager.encode_sensor_message(sensor_id, sensor_value))

    @classmethod
    def encode_sensor_message(cls, sensor_id, value):
        return json.dumps(dict(sensor=sensor_id, pressure=value))

    @classmethod
    def decode_sensor_message(cls, serialized):
        decoded_dict = json.loads(serialized)
        try:
            return int(decoded_dict['sensor']), int(decoded_dict['pressure'])
        except (ValueError, KeyError) as e:
            return None, None

    def check_failures(self, sensor_id, sensor_value, fail_ceil, fail_floor):
        if sensor_value > fail_ceil or sensor_value < fail_floor:
            self.failure_publisher.publish("Sensor: %d" % (sensor_id))
            return True
        return False

    def publish_failure(self, sensor_id):
        self.failure_publisher.publish("%d" % sensor_id)

    def sleep(self):
        rospy.Rate(1000).sleep()

if __name__ == '__main__':
   print "Not implemented"