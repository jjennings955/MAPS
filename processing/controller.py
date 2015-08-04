from communication import readSerial
from events import EventManager
from aggregation import AggregationBuffer, aggregation_initialize, aggregation_update
from calibration import set_threshholds, set_threshhold
from failuredetect import *

class Controller(object):
    def __init__(self, num_sensors=16, aggregation_window=10):
        self.eventManager = EventManager()
        self.num_sensors = num_sensors
        self.aggregation_buffers = [AggregationBuffer(aggregation_window) for i in range(num_sensors)]
        set_threshholds(dict(zip([str(i) for i in range(num_sensors)], [0 for i in range(num_sensors)]))) # Initialize all threshholds to 0
        aggregation_initialize(16)

    def run(self):
        try:
            #set_threshholds(dict(zip([str(i) for i in range(16)], [0 for i in range(16)])))
            #eventManager = EventManager()
            while True:
                val, pin_id = readSerial()
                if pin_id < 0 or pin_id >= self.num_sensors:
                    continue
                aggregation_update(pin_id, val)
                floor_thresh = get_floor_threshold(pin_id)
                ceil_thresh = get_ceil_threshold(pin_id)
                self.eventManager.check_failures(pin_id, val, ceil_thresh, floor_thresh)
                self.eventManager.publish_packet(pin_id, val)
                self.aggregation_buffers[pin_id].add(val)
                self.eventManager.sleep()

        except rospy.ROSInterruptException:
            pass


if __name__ == "__main__":
    controller = Controller()
    controller.run()