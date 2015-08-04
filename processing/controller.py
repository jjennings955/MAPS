from communication import readSerial
from events import EventManager
from calibration import set_threshholds, set_threshhold


if __name__ == "__main__":
    print "Unuused. Run events.py"
    # set_threshholds(dict(zip([str(i) for i in range(10)], [400 for i in range(10)])))
    # set_threshhold(0, 0)
    # eventManager = EventManager()
    # while True:
    #     value, id = readSerial()
    #     print(value, id)
    #     eventManager.publish_packet(id, value)
    #     eventManager.sleep()
