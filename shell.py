######################################################################
#
#   M.A.P.S Utilty General User Interface
#
#   Coded By:     Jason Jennings and Marcus Mirzaie
#   Designed By:  Timothy Beene, Jason Jennings, Marcus Mirzaie,
#                 Khanh Ngo, and Ninh Nguyen
#
######################################################################
import rospy
rospy.init_node("MAPS_GUI")

try:
    from presentation.calibration import MAPSInterface
    from processing.calibration import get_threshhold
    from processing.aggregation import get_current_value
    DEBUG = False
except:
    from presentation.calibrationdummy import *
    DEBUG = True
    def get_threshhold(which):
        return 0.0
    def get_current_value(which):
        return 0.0


from presentation.recording import DataRecorder
from Tkinter import *
from presentation.layout import Sensor, read_layout_file

class DemonstrationGui(object):
    def __init__(self, layout_file='configuration\layout.csv'):
        self.master = Tk()
        self.master.wm_title("MAPS Demonstration GUI")

        # Interfaces to external programs
        self.data_recorder = DataRecorder(topics=["/pressure_events"])
        self.command_interface = MAPSInterface()

        self.sensor_objs = []
        self.sensor_widgets = {}

        # State Variables for Widgets
        self.filter_state = IntVar()
        self.record_button_text = StringVar()
        self.record_button_text.set("Record Data")

        # Declare Widgets
        self.calibrate_button = Button(self.master, text="Calibrate", command=self.calibrate_button_callback)
        self.record_button = Button(self.master, textvariable=self.record_button_text, command=self.record_callback)
        self.filter_button = Checkbutton(self.master, text="Filter", variable=self.filter_state)
        self.threshold_label = Label(self.master, text="Threshold(%):  ")
        self.threshold_slider = Scale(self.master, from_=0.0, to=100.0, orient=HORIZONTAL)

        # Read Layout File and create Sensor Widgets
        self.read_layout(layout_file)

        # Position widgets
        self.calibrate_button.grid(row=0, column=Sensor.max_width+1)
        self.record_button.grid(row=1, column=Sensor.max_width+1)
        self.filter_button.grid(row=2, column=Sensor.max_width+1)
        self.threshold_label.grid(row=3, column=Sensor.max_width+1)
        self.threshold_slider.grid(row=4, column=Sensor.max_width+1)

    def read_layout(self, fname):
        self.sensor_objs = read_layout_file(fname)
        self.sensor_widgets = {}
        for idx, sensor in self.sensor_objs.iteritems():
            canvas = sensor.create_canvas(self.master)
            canvas.grid(row=sensor.row, column=sensor.col, padx=1, pady=1)
            self.sensor_widgets[idx] = sensor

    def update_timer(self):
        for ind, lbl in self.sensor_widgets.iteritems():
            self.sensor_widgets[ind].update_canvas(get_current_value(int(ind)), get_threshhold(int(ind)))
        self.master.after(10, self.update_timer)

    def record_callback(self):
        if not self.data_recorder.recording:
            self.data_recorder.start()
            self.record_button_text.set("Stop Recording")
        else:
            self.data_recorder.stop()
            self.record_button_text.set("Record Data")

    def ros_event_callback(self, data):
        pass

    def calibrate_button_callback(self):
        self.command_interface.calibrate(self.threshold_slider.get()/100.0)

    def run(self):
        self.update_timer()
        self.master.mainloop()

# ROS CALLBACK
def update_value(data):
    results = re.findall(r"Sensor: ([0-9]+), Pressure: ([0-9.]+)", data.data)
    if results:
        ind = int(results[0][0])
        val = float(results[0][1])
        pass # REPLACE ME

if __name__ == "__main__":
    # Run GUI
    gui = DemonstrationGui()
    gui.run()
