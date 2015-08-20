import os

import rospy
from std_msgs.msg import String


rospy.init_node("MAPS_GUI")

try:
    from presentation.calibration import MAPSInterface
    from processing.calibration import get_threshold
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
        """
        A GUI to demonstrate the basic functionality of the system.
        :param layout_file:
        :return:
        """
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
        #self.filter_button = Checkbutton(self.container, text="Filter", variable=self.filter_state)
        self.threshold_label = Label(self.container, text="Threshold(%):  ")
        self.threshold_slider = Scale(self.container, from_=0.0, to=100.0, orient=HORIZONTAL)

        # Position widgets within container
        self.calibrate_button.grid(row=0, column=0, pady=10)
        self.threshold_label.grid(row=2, column=0, pady=10)
        self.threshold_slider.grid(row=4, column=0, pady=10)
        self.record_button.grid(row=6, column=0, pady=10)
        #self.filter_button.grid(row=8, column=0, pady=10)

        # Add container to window
        self.container.grid(row=0, column=Sensor.max_width+1, rowspan=5)

    def read_layout(self, fname):
        """
        Read the layout file to create the grid positioning the sensor widgets
        :param fname: The file name of the layout file to read
        :return: None
        """
        self.sensor_objs = read_layout_file(fname)
        self.sensor_widgets = {}
        for idx, sensor in self.sensor_objs.iteritems():
            canvas = sensor.create_canvas(self.master)
            canvas.grid(row=sensor.row, column=sensor.col, padx=1, pady=1)
            self.sensor_widgets[idx] = sensor

    def update_timer(self):
        """
        The timer that updates the widgets
        :return: None
        """
        for ind, lbl in self.sensor_widgets.iteritems():
            self.sensor_widgets[ind].update_canvas(get_current_value(int(ind)), get_threshold(int(ind)))
        self.master.after(10, self.update_timer)

    def record_callback(self):
        """
        The callback function called when the "record" button is clicked
        :return:
        """
        if not self.data_recorder.recording:
            self.data_recorder.start()
            self.record_button_text.set("Stop Recording")
        else:
            self.data_recorder.stop()
            self.record_button_text.set("Record Data")

    def ros_event_callback(self, data):
        """
        The callback function called when a ros pressure event is received
        :param data: Pressure data
        :return:
        """
        sensor, value = EventManager.decode_sensor_message(data.data)
        if sensor is not None:
            sensor = str(sensor)
            self.sensor_widgets[sensor].trigger()
            self.master.after(100, self.sensor_widgets[sensor].remove_trigger)

    def pocket_fail_callback(self, data):
        """
        The callback function called when a pocket fails
        :param data: The sensor that failed
        :return:
        """
        sensor = data.data
        self.sensor_widgets[sensor].fail()
        self.master.after(100, self.sensor_widgets[sensor].remove_fail)

    def calibrate_button_callback(self):
        """
        The callback function called when the calibrate button is pressed
        :return: None
        """
        self.command_interface.calibrate(self.threshold_slider.get()/100.0)

    def run(self):
        """
        Start the event loop for the GUI
        :return: None
        """
        rospy.Subscriber("/pressure_events", String, self.ros_event_callback)
        rospy.Subscriber("/failure_events", String, self.pocket_fail_callback)
        self.update_timer()
        self.master.mainloop()

if __name__ == "__main__":
    # Run GUI
    gui = DemonstrationGui(layout_file=os.path.join('configuration', 'layout.csv'))
    gui.run()
