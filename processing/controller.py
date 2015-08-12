
from communication import Communication
#from comm_dummy import readSerial

from events import EventManager
from aggregation import AggregationBuffer, aggregation_initialize, aggregation_update
from calibration import set_threshholds, set_threshhold
from failuredetect import *
from filtering import apply_filter
import numpy as np
from configuration import SERIAL_PORT

class Controller(object):
    def __init__(self, num_sensors=16, aggregation_window=5):
        self.eventManager = EventManager()
        self.num_sensors = num_sensors
        self.aggregation_buffers = [AggregationBuffer(aggregation_window) for i in range(num_sensors)]
        default_threshhold_dict = dict(zip([str(i) for i in range(num_sensors)], [0 for i in range(num_sensors)]))
        set_threshholds(default_threshhold_dict) # Initialize all threshholds to 0
        aggregation_initialize(16)

    def valid_pin(self, number):
        return number >= 0 and number <= self.num_sensors

    def run(self):
        comm = Communication(SERIAL_PORT)
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
                self.eventManager.sleep()

        except rospy.ROSInterruptException:
            pass


if __name__ == "__main__":
    controller = Controller()
    controller.run()