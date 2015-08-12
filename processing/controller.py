
from communication import Communication

from events import EventManager
from aggregation import AggregationBuffer, aggregation_initialize, aggregation_update
from calibration import set_threshhold
from failuredetect import auto_fail_absolute, get_floor_threshold, get_ceil_threshold
from filtering import apply_filter
import numpy as np
from configuration import SERIAL_PORT
import serial
import time

class Controller(object):
    def __init__(self, num_sensors=16, aggregation_window=5):
        self.eventManager = EventManager()
        self.num_sensors = num_sensors
        self.aggregation_buffers = [AggregationBuffer(aggregation_window) for i in range(num_sensors)]

        # Initialize all thresholds to zero by default
        for i in range(num_sensors):
            set_threshhold(i, 0)
        auto_fail_absolute()
        aggregation_initialize(num_sensors)

    def valid_pin(self, number):
        return number >= 0 and number <= self.num_sensors

    def run(self):
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
                ceil_thresh = 50
                if not self.eventManager.check_failures(pin_id, val, ceil_thresh, floor_thresh):
                    # no failures
                    vals = self.aggregation_buffers[pin_id].history.T
                    filtered_val = apply_filter(vals, weights)
                    #aggregation_update(pin_id, filtered_val)
                    self.eventManager.publish_packet(pin_id, val)
                    self.aggregation_buffers[pin_id].add(val)
                else:
                    self.eventManager.publish_failure(pin_id)

                self.eventManager.sleep()

        except rospy.ROSInterruptException:
            pass


if __name__ == "__main__":
    controller = Controller()
    while True:
        try:
            controller.run()
        except serial.serialutil.SerialException as e:
            controller.eventManager.sleep()
            continue
