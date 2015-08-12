######################################################################
#
#   M.A.P.S Utilty General User Interface
#
#   Coded By:     Marcus Mirzaie
#   Designed By:  Timothy Beene, Jason Jennings, Marcus Mirzaie,
#                 Khanh Ngo, and Ninh Nguyen
#
######################################################################
import rospy
rospy.init_node("MAPS_GUI")
try:
    from calibration import MAPSInterface
    DEBUG = False
    def get_threshhold(which):
        return rospy.get_param('calibration_threshold/%d' % (which), 0)

    def get_current_value(which):
        return rospy.get_param('current_value/%d' % which, 0)
except:
    from calibrationdummy import *
    DEBUG = True
    def get_threshhold(which):
        return 0.0
    def get_current_value(which):
        return 0.0


import re
from communication import *
from recording import DataRecorder
from Tkinter import *
from fileinput import Sensor, read_layout_file
# Intermediary Functions


interface = MAPSInterface()

def calibrate_button():
    interface.calibrate(ThresholdPercent.get())
    #calibrate(ThresholdPercent.get())

# UNUSED?
def get_sensor_value(ind):
    return update_sensor(ind)




# Make Main Window
master = Tk()
master.wm_title('MAPS')

# Initialize Sensor Values
SensorValue = []
SensorLabel = []
SensorThreshold = []

# Draw Threshold EntryBox
Label(master, text="Threshold(%):  ").grid(column=2, row=0)
ThresholdPercent = Scale(master, from_=0.0, to=100.0, orient=HORIZONTAL)
#ThresholdPercent.grid(column=2, row=1)
RecordButtonText = StringVar()
RecordButtonText.set("Record Data")
data_recorder = DataRecorder(topics=["/pressure_events"])

def record_button():
    global recording, pro, RecordButton
    if not data_recorder.recording:
        data_recorder.start()
        RecordButtonText.set("Stop Recording")
    else:
        data_recorder.stop()
        RecordButtonText.set("Record Data")


def get_thresh_str(sensor):
    return "Thresh:  %.2f" % get_threshhold(sensor)

sensor_objs = read_layout_file('whatever.csv')
sensor_widgets = {}
for id, sensor in sensor_objs.iteritems():
    canvas = sensor.create_canvas(master)
    canvas.grid(row=sensor.row, column=sensor.col, padx=1, pady=1)
    sensor_widgets[id] = sensor
    sensor.update_canvas(10)



def update_values():
     for ind, lbl in sensor_widgets.iteritems():
         print(get_current_value(int(ind)))
         sensor_widgets[ind].update_canvas(get_current_value(int(ind)), get_threshhold(int(ind)))
     master.after(10, update_values)

# ROS CALLBACK
def update_value(data):
    results = re.findall(r"Sensor: ([0-9]+), Pressure: ([0-9.]+)", data.data)
    if results:
        ind = int(results[0][0])
        val = float(results[0][1])
        SensorValue[ind] = val
        tmp2 = str(ind+1) + ":  " + str(SensorValue[ind])[:6] + " PSI" + "\n" + get_thresh_str(ind)
        SensorLabel[ind].config(text=tmp2)


# Draw Calibrate Button
CalibrateButton = Button(master, text="Calibrate", command=calibrate_button)
CalibrateButton.grid(row=0, column=sensor.max_width+1)
# CalibrateButton.pack()

# Draw Record Data Checkbox
RecordButton = Button(master, textvariable=RecordButtonText, command=record_button)
RecordButton.grid(row=1, column=sensor.max_width+1)
ThresholdPercent.grid(column=sensor.max_width+1, row=3)
# Draw Filter Checkbox
FilterCheckbox = IntVar()
Checkbutton(master, text="Filter", variable=FilterCheckbox).grid(row=2, column=sensor.max_width+1)

if __name__ == "__main__":
    # Run GUI
    foo = Sensor(0, 0, 0)
    if not DEBUG:
        pass
    import rospy
    #rospy.init_node('MAPS_listener')

    update_values()

    #rospy.Subscriber("/pressure_events", String, update_value)
       # rospy.spin()
    master.mainloop()
