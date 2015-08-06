__author__ = 'jason'
class Sensor(object):
    def __init__(self, sensor, x, y):
        self.sensor_id = sensor
        self.x = x
        self.y = y
    def __repr__(self):
        return "Sensor(id=%d, x=%d, y=%d)" % (self.sensor_id, self.x, self.y)
    def __str__(self):
        return self.__repr__()
    def draw(self):
        pass

def read_layout_file(fname):
    sensors = []
    try:
        with open(fname, 'r') as fd:
            for line in fd:
                sensor_id, x_position, y_position = line.split(',')
                sensors.append(Sensor(int(sensor_id), int(x_position), int(y_position)))
    except (ValueError, IOError) as e:
        print "Ignoring bad line in file %s" % (fname)
    return sensors

if __name__ == "__main__":
    sensor = read_layout_file("whatever.csv")
    print(sensor)