import rospy
from std_msgs.msg import String
from calibration import  get_threshold, auto_calibrate
from filtering import set_filter, FILTER_FIR, FILTER_MEDIAN, FILTER_NONE
import json

class EventManager(object):
    def __init__(self):
        """
        Declare a new EventManager object for sending ROS commands
        """
        rospy.init_node('MAPS_talker', anonymous=True)
        self.publisher = rospy.Publisher('pressure_events', String, queue_size=1)
        self.command_manager = rospy.Subscriber("MAPScommand", String, self.handle_event)
        self.failure_publisher = rospy.Publisher('failure_events', String, queue_size=1)

    def handle_event(self, data):
        """
        Callback function for MAPScommand topic
        :param data: The command data sent with MAPScommand topic
        """
        command, arg = data.data.split(' ')
        if command == 'autocalibrate':
            auto_calibrate(float(arg))
        elif command == 'enable_fir':
            set_filter(FILTER_FIR)
        elif command == 'enable_median':
            set_filter(FILTER_MEDIAN)
        elif command == 'disable_filter':
            set_filter(FILTER_NONE)

    def check_threshold(self, sensor_id, sensor_value):
        """
        Check if a sensor value exceeds its calibrated threshold for the specified sensor
        :param sensor_id: The sensor to check
        :param sensor_value: The value to check
        """
        threshold = get_threshold(sensor_id)
        return sensor_value > threshold

    def publish_packet(self, sensor_id, sensor_value, debug=False):
        """
        Publish a pressure_event packet to the ROS system
        :param sensor_id: The sensor ID to publish a packet for
        :param sensor_value: The pressure reading to publish
        :param debug: A flag for debugging purpose, will publish events regardless of calibration
        """
        if debug or self.check_threshold(sensor_id, sensor_value):
            self.publisher.publish(EventManager.encode_sensor_message(sensor_id, sensor_value))

    @classmethod
    def encode_sensor_message(cls, sensor_id, value):
        """
        Helper method to encode a sensor message using JSON
        :param sensor_id: The sensor ID to encode
        :param value: The value to encode
        """
        return json.dumps(dict(sensor=sensor_id, pressure=str(value)))

    @classmethod
    def decode_sensor_message(cls, serialized):
        """
        Helper method to decode a serialized sensor message into its components
        :param serialized: The serialized data to decode
        :returns (sensor_id, PSI_value) if valid, otherwise (None, None)
        """
        decoded_dict = json.loads(serialized)
        try:
            return int(decoded_dict['sensor']), float(decoded_dict['pressure'])
        except (ValueError, KeyError) as e:
            return None, None

    def check_failures(self, sensor_id, sensor_value, fail_ceil, fail_floor):
        """
        Check if a sensor reading corresponds to a failure
        :param sensor_id: The sensor ID to check
        :param sensor_value: The sensor value to check
        :param fail_ceil: The maximum value beyond which a failure is triggered
        :param fail_floor: The minimum value below which a failure is triggered
        :returns True if a failure is detected, False if not
        """
        if sensor_value > fail_ceil or sensor_value < fail_floor:
            self.publish_failure(sensor_id)
            return True
        return False

    def publish_failure(self, sensor_id):
        """
        Publish a failure event on the specified sensor
        :param sensor_id: The sensor on which a failure was detected
        """
        self.failure_publisher.publish("%d" % sensor_id)

    def sleep(self):
        rospy.Rate(1000).sleep()

if __name__ == '__main__':
   print "Not implemented"