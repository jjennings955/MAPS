from Tkinter import ALL, W, Canvas

class Sensor(object):
    max_width = 0
    max_height = 0
    WIDTH = 150
    HEIGHT = 100
    MULTIPLIER = 30

    def __init__(self, sensor, row, col, val=0):
        self.sensor_id = sensor
        self.row = row
        self.col = col
        self.canvas = None
        self.value = val
        self.triggered = 0
        self.failed = 0

        if self.col > Sensor.max_width:
            Sensor.max_width = self.col
        if self.row > Sensor.max_height:
            Sensor.max_height = self.row

    def __repr__(self):
        return "Sensor(id=%d, row=%d, col=%d)" % (self.sensor_id, self.row, self.col)

    def __str__(self):
        return self.__repr__()

    def create_canvas(self, master):
        self.canvas = Canvas(master, width=Sensor.WIDTH, height=Sensor.HEIGHT)
        self.update_canvas(0.0)
        return self.canvas

    def update_canvas(self, value, thresh=14.0):
        self.value = value
        self.thresh = thresh
        if self.triggered > 0:
            bar_color = 'red'
        else:
            bar_color = 'blue'

        self.canvas.delete(ALL)
        self.canvas.create_rectangle(2, 2, Sensor.WIDTH, Sensor.HEIGHT, outline='black')

        if self.failed <= 0:
            self.canvas.create_rectangle(2, min(Sensor.HEIGHT, Sensor.HEIGHT - Sensor.MULTIPLIER*(self.value - 12)), Sensor.WIDTH, Sensor.HEIGHT, fill=bar_color)
            self.canvas.create_line(2, min(Sensor.HEIGHT, Sensor.HEIGHT - Sensor.MULTIPLIER*(self.thresh - 12)),
                        Sensor.WIDTH, min(Sensor.HEIGHT, Sensor.HEIGHT -  Sensor.MULTIPLIER*(self.thresh - 12)),
                        fill='red')

            self.canvas.create_text(Sensor.WIDTH/2, 7.0*Sensor.HEIGHT/8, text="%.3f" % self.value, font=('Courier', '8', 'bold'), fill='black')
        else:
            self.canvas.create_line(2, 2, Sensor.WIDTH, Sensor.HEIGHT, fill='red', width=3)
            self.canvas.create_line(Sensor.WIDTH, 2, 0, Sensor.HEIGHT, fill='red', width=3)
        self.canvas.create_text(Sensor.WIDTH/2, Sensor.HEIGHT/4, text="%d" % (self.sensor_id), font=('Courier', '12', 'bold'))

    def trigger(self):
        self.triggered += 1
        self.triggered = min(self.triggered, 10)

    def fail(self):
        self.failed += 1
        self.failed = min(self.failed, 10)

    def remove_trigger(self):
        self.triggered -= 1
        self.triggered = max(self.triggered, 0)
        self.update_canvas(self.value, self.thresh)

    def remove_fail(self):
        self.failed -= 1
        self.failed = max(self.failed, 0)
        self.update_canvas(self.value, self.thresh)

