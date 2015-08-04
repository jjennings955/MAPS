import rospy
import numpy as np

def set_windowsize(n=10):
    """Maintain an internal history of n sensor readings"""
    rospy.set_param('aggregation_windowsize', n)
def get_current_values():
    return rospy.get_param('current_value', {})
def initialize(n):
    for i in range(n):
        rospy.set_param('current_value/%d' % i, 0)

def update(which, val):
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