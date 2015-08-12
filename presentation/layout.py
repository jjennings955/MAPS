from widgets import Sensor

def read_layout_file(fname):
    sensors = {}
    try:
        with open(fname, 'r') as fd:
            for row, line in enumerate(fd):
                entries = line.split(',')
                for col, entry in enumerate(entries):
                    entry = entry.strip()
                    if entry:
                        sensors[entry] = Sensor(int(entry), int(row), int(col))
    except (ValueError, IOError) as e:
        print "Ignoring bad line in file %s" % (fname)
    return sensors

if __name__ == "__main__":
    sensor = read_layout_file("layout.csv")
    print(sensor)
    print(Sensor.max_height + 1, " ", Sensor.max_width + 1)
