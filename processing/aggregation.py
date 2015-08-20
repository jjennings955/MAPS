import rospy
import numpy as np

def set_windowsize(n=10):
    """
    Set the size of the history to maintain for each sensor
    :param n: The number of values to keep
    :return: None
    """
    rospy.set_param('aggregation_windowsize', n)


def get_current_values():
    """
    Get the current value of all sensors
    :return: A map of sensor ids to current values
    """
    return rospy.get_param('current_value', {})


def get_current_value(which):
    """
    Get the current value of the specified sensor
    :param which: The sensor to retrieve a value for
    :return: The value of the current sensor in PSI
    """
    return rospy.get_param('current_value/%d' % which)


def aggregation_initialize(n):
    """
    Initialize the aggregation system, setting the current value of all sensors to 0
    :param n: The number of sensors to initalize
    :return: None
    """
    for i in range(n):
        rospy.set_param('current_value/%d' % i, 0)


def aggregation_update(which, val):
    """
    Update the current value for the specified sensor
    :param which: The sensor to update
    :param val: The value to set it to
    :return: None
    """
    rospy.set_param('current_value/%d' % which, val)


class AggregationBuffer(object):
    def __init__(self, window_size=1):
        self.history = np.zeros(dtype=np.float32, shape=(window_size,1))
        self.window_size = window_size
        self.head_ptr = 0
        self.tail_ptr = 0
        self.full = False
        self.empty = True

    def add(self, item):
        self.empty = False
        if not self.full:
            self.history[self.tail_ptr] = item
            self.tail_ptr += 1
            if self.tail_ptr == self.window_size:
                self.full = True
        else:
            self.history = np.roll(self.history, -1)
            self.history[self.window_size - 1] = item

    def window(self):
        if self.empty:
            return None
        if self.full:
            return self.history
        else:
            return self.history[0:self.tail_ptr]

    def mean(self):
        window = self.window()
        if window is not None:
            return np.mean(window)

    def median(self):
        window = self.window()
        if window is not None:
            return np.median(window)

    def min(self):
        window = self.window()
        if window is not None:
            return np.min(window)

    def max(self):
        window = self.window()
        if window is not None:
            return np.max(window)



if __name__ == "__main__":
    foo = AggregationBuffer(10)
    print(foo.mean())
    print(foo.median())
    print(foo.min())
    print(foo.max())

    for i in range(10):
        foo.add(i)
        print(foo.history)
    for i in range(5):
        foo.add(i)
    print foo.mean()
    print foo.median()
    print foo.min()
    print foo.max()
