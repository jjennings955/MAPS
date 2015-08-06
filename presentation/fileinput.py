__author__ = 'jason'

class Sensor(object):
    cell_height = 0
    cell_width = 0
    max_width = 0
    max_height = 0
    def __init__(self, sensor, row, col):
        self.sensor_id = sensor
        self.row = row
        self.col = col
        if self.col > Sensor.max_width:
            Sensor.max_width = self.col
        if self.row > Sensor.max_height:
            Sensor.max_height = self.row
    def __repr__(self):
        return "Sensor(id=%d, row=%d, col=%d)" % (self.sensor_id, self.row, self.col)
    def __str__(self):
        return self.__repr__()
    def draw(self):
        pass
    def draw_canvas(self):
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
    print(Sensor.max_height + 1, " ", Sensor.max_width + 1)
