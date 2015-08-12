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
import os
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

from processing.events import EventManager
from presentation.recording import DataRecorder
from Tkinter import *
from presentation.layout import Sensor, read_layout_file

class DemonstrationGui(object):
    def __init__(self, layout_file):
        self.master = Tk()
        self.master.wm_title("MAPS Demonstration GUI")

        # Interfaces to external programs
        self.data_recorder = DataRecorder(topics=["/pressure_events"])
        self.command_interface = MAPSInterface()

        self.sensor_objs = []
        self.sensor_widgets = {}

        # Read Layout File and create Sensor Widgets
        self.read_layout(layout_file)

        # State Variables for Widgets
        self.filter_state = IntVar()
        self.record_button_text = StringVar()
        self.record_button_text.set("Record Data")

        # Declare container for command/config widgets
        self.container = Frame(self.master)

        # Declare Widgets
        self.calibrate_button = Button(self.container, text="Calibrate", command=self.calibrate_button_callback)
        self.record_button = Button(self.container, textvariable=self.record_button_text, command=self.record_callback)
        self.filter_button = Checkbutton(self.container, text="Filter", variable=self.filter_state)
        self.threshold_label = Label(self.container, text="Threshold(%):  ")
        self.threshold_slider = Scale(self.container, from_=0.0, to=100.0, orient=HORIZONTAL)

        # Position widgets within container
        self.calibrate_button.grid(row=0, column=0, pady=10)
        self.record_button.grid(row=2, column=0, pady=10)
        self.filter_button.grid(row=4, column=0, pady=10)
        self.threshold_label.grid(row=6, column=0, pady=10)
        self.threshold_slider.grid(row=8, column=0, pady=10)

        # Add container to window
        self.container.grid(row=0, column=Sensor.max_width+1, rowspan=5)

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
        info = EventManager.decode_sensor_message(data.data)
        print(info)


    def calibrate_button_callback(self):
        self.command_interface.calibrate(self.threshold_slider.get()/100.0)

    def run(self):
        self.update_timer()
        self.master.mainloop()

if __name__ == "__main__":
    # Run GUI
    gui = DemonstrationGui(layout_file=os.path.join('configuration', 'layout.csv'))
    gui.run()
