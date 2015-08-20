import numpy as np
import serial
import rospy

from processing.communication import Communication
from processing.events import EventManager
from processing.aggregation import AggregationBuffer, aggregation_initialize, aggregation_update
from processing.calibration import set_threshhold
from processing.failuredetect import auto_fail_absolute, get_floor_threshold, get_ceil_threshold
from processing.filtering import apply_filter, filtering_initialize, get_filter_mode, FILTER_NONE

from configuration.settings import SERIAL_PORT, AGGREGATION_WINDOW, NUM_SENSORS, FAIL_CEIL, FAIL_FLOOR


class Controller(object):

    def __init__(self, num_sensors=16, aggregation_window=5, fail_floor=12.0, fail_ceil=20.0):
        """
        Create a new controller object
        :param num_sensors: The number of connected sensors
        :param aggregation_window: The number of values to maintain in the aggregation system
        :param fail_floor: The lower bound on reasonable values
        :param fail_ceil: The upper bound on reasonable values
        """
        self.eventManager = EventManager()
        self.num_sensors = num_sensors
        self.aggregation_buffers = [AggregationBuffer(aggregation_window) for i in range(num_sensors)]

        # Initialize all thresholds to zero by default
        for i in range(num_sensors):
            set_threshhold(i, 0)

        auto_fail_absolute(fail_floor, fail_ceil)
        aggregation_initialize(num_sensors)
        filtering_initialize()

    def valid_pin(self, number):
        """
        Check if a pin number is valid
        :param number: The number to check
        :return: True if it's a valid pin id, False if it is not
        """
        return (number >= 0) and (number < self.num_sensors)

    def run(self):
        """
        Run the controller event loop
        :return:
        """
        comm = Communication(SERIAL_PORT)
        print("Running MAPS controller on serial port: %s" % (SERIAL_PORT))
        try:
            weights = np.ones(dtype=np.float32, shape=(5,1))/5
            while True:
                val, pin_id = comm.read_packet()
                if not self.valid_pin(pin_id):
                    continue
                aggregation_update(pin_id, val)
                floor_thresh = get_floor_threshold(pin_id)
                ceil_thresh = get_ceil_threshold(pin_id)
                if not self.eventManager.check_failures(pin_id, val, ceil_thresh, floor_thresh):
                    # no failures
                    vals = self.aggregation_buffers[pin_id].history.T
                    if get_filter_mode() != FILTER_NONE:
                        filtered_val = apply_filter(vals)
                    else:
                        filtered_val = val

                    self.eventManager.publish_packet(pin_id, filtered_val)
                    self.aggregation_buffers[pin_id].add(val)
                else:
                    self.eventManager.publish_failure(pin_id)

                self.eventManager.sleep()

        except rospy.ROSInterruptException:
            pass


if __name__ == "__main__":
    controller = Controller(num_sensors=NUM_SENSORS, aggregation_window=AGGREGATION_WINDOW, fail_floor=FAIL_FLOOR, fail_ceil=FAIL_CEIL)
    while True:
        try:
            controller.run()
        except serial.serialutil.SerialException as e:
            controller.eventManager.sleep()
            continue
