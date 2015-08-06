__author__ = 'jason'
class Sensor(object):
    def __init__(self, sensor, row, col):
        self.sensor_id = sensor
        self.row = row
        self.col = col
    def __repr__(self):
        return "Sensor(id=%d, row=%d, col=%d)" % (self.sensor_id, self.row, self.col)
    def __str__(self):
        return self.__repr__()
    def draw(self):
        pass

def read_layout_file(fname):
    sensors = []
    try:
        with open(fname, 'r') as fd:
            for row, line in enumerate(fd):
                entries = line.split(',')
                for col, entry in enumerate(entries):
                    entry = entry.strip()
                    if entry:
                        sensors.append(Sensor(int(entry), int(row), int(col)))
    except (ValueError, IOError) as e:
        print "Ignoring bad line in file %s" % (fname)
    return sensors

if __name__ == "__main__":
    sensor = read_layout_file("whatever.csv")
    print(sensor)