######################################################################
#
#   M.A.P.S Utilty General User Interface
#
#   Coded By:     Marcus Mirzaie
#   Designed By:  Timothy Beene, Jason Jennings, Marcus Mirzaie,
#                 Khanh Ngo, and Ninh Nguyen
#
######################################################################
try:
    from calibration import *
except:
    from calibrationdummy import *

import subprocess
from communication import *
from recording import DataRecorder
from Tkinter import *




# Intermediary Functions


def calibrate_button():
    #tkMessageBox.showinfo("Calibrate", "Calibrating...")
    calibrate(ThresholdPercent.get())

# UNUSED?
def get_sensor_value(ind):
    return update_sensor(ind)




# Make Main Window
master = Tk()

# Initialize Sensor Values
SensorValue = []
SensorLabel = []
SensorThreshold = []

# Draw Threshold EntryBox
Label(master, text="Threshold(%):  ").grid(column=2, row=0)
ThresholdPercent = Scale(master, from_=0.0, to=100.0, orient=HORIZONTAL)
ThresholdPercent.grid(column=2, row=1)
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
    return "Thresh:  " + str(SensorThreshold[sensor] * float(ThresholdPercent.get()) / 100)

for i in range(0, 16):
    SensorValue.append(0.0)
    SensorThreshold.append(0.0)
    if i >= 8:
        tmp = str(i+1) + ":  " + str(SensorValue[i]) + " PSI" + "\n" + get_thresh_str(i)
        SensorLabel.append(Label(master, text=tmp))
        SensorLabel[i].grid(row=i-8, column=1, padx=5, pady=5)
    else:
        tmp = str(i+1) + ":  " + str(SensorValue[i]) + " PSI" + "\n" + get_thresh_str(i)
        SensorLabel.append(Label(master, text=tmp))
        SensorLabel[i].grid(row=i, column=0, padx=5, pady=5)

# UNUSED?
def update_values():
    for ind, lbl in enumerate(SensorLabel):
        sval = get_sensor_value(ind)
        tmp2 = str(ind+1) + ":  " + str(SensorValue[ind]) + " PSI" + "\n" + get_thresh_str(ind)
        lbl.config(text=tmp2)
#master.after(100, update_values)


# ROS CALLBACK
def update_value(data):
    results = re.findall(r"Sensor: ([0-9]+), Pressure: ([0-9.]+)", data.data)
    if results:
        ind = int(results[0][0])
        val = float(results[0][1])
        SensorValue[ind] = val
        tmp2 = str(ind+1) + ":  " + str(SensorValue[ind]) + " PSI" + "\n" + get_thresh_str(ind)
        SensorLabel[ind].config(text=tmp2)


# Draw Calibrate Button
CalibrateButton = Button(master, text="Calibrate", command=calibrate_button)
CalibrateButton.grid(row=2, column=2)
# CalibrateButton.pack()

# Draw Record Data Checkbox
RecordButton = Button(master, textvariable=RecordButtonText, command=record_button)
RecordButton.grid(row=3, column=2)

# Draw Filter Checkbox
FilterCheckbox = IntVar()
Checkbutton(master, text="Filter", variable=FilterCheckbox).grid(row=4, column=2)

if __name__ == "__main__":
    # Run GUI
    try:
        import rospy
        rospy.init_node('MAPS_listener')
        rospy.Subscriber("/pressure_events", String, update_value)
    except:
        pass
    #rospy.spin()
    master.mainloop()
