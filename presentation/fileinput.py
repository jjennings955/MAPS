__author__ = 'jason'

class Sensor(object):
    cell_height = 0
    cell_width = 0
    max_width = 0
    max_height = 0
    def __init__(self, sensor, row, col, val=0):
        self.sensor_id = sensor
        self.row = row
        self.col = col
        self.canvas = None
        self.value = val
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
    def create_canvas(self, master):
        from Tkinter import Canvas
        import random
        self.canvas = Canvas(master, width=75, height=50)
        self.thresh = random.randint(13, 16)
        #self.update_canvas()
        return self.canvas

    def update_canvas(self, value, thresh=14.0):
        #import random
        from Tkinter import ALL, W
        #self.value = 0.8*self.value + 0.2*random.randint(0,36) # EXAMPLE ONLY!?!?!?%#%#
        self.value = value
        self.thresh = thresh
        self.canvas.delete(ALL)
        self.canvas.create_rectangle(0, 50 - 10*(self.value - 12), 50, 50, fill='blue')
        self.canvas.create_line(0, 50 - 10*(self.thresh - 12), 50, 50 - 10*(self.thresh - 12), fill='red')
        self.canvas.create_rectangle(2, 2, 50, 50, outline='black')
        self.canvas.create_text(5, 35, text="%.3f" % self.value, font=('Courier', '8', 'bold'), anchor=W, fill='green')
        self.canvas.create_text(55, 35, text="%d" % (self.sensor_id), anchor=W)
        #self.canvas.after(100, self.update_canvas)


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
    sensor = read_layout_file("whatever.csv")
    print(sensor)
    print(Sensor.max_height + 1, " ", Sensor.max_width + 1)
