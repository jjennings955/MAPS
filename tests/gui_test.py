__author__ = 'cseflqsn22'

import sys
import time
sys.path.insert(0,'..')
from processing.calibration import set_threshhold
from processing.aggregation import aggregation_update
from processing.events import EventManager

def test_all():
    event_manager = EventManager()

    for i in range(16):
        set_threshhold(i, 0)
        aggregation_update(i, 0)
    test_range = range(-32,32) + range(32,0, -1)
    for i in range(16):
        for j in test_range:
            set_threshhold(i, j)
            time.sleep(0.01)

        for k in test_range:
            aggregation_update(i, k)
            time.sleep(0.01)

        set_threshhold(i, 15.0)
        aggregation_update(i, 16.0)
        time.sleep(0.01)
        event_manager.publish_packet(i, 16.0)

        time.sleep(0.5)
        aggregation_update(i, 11.0)
        event_manager.publish_failure(i)
        time.sleep(0.5)
