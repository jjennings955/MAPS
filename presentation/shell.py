######################################################################
#
#   M.A.P.S Utilty General User Interface
#
#   Coded By:     Marcus Mirzaie
#   Designed By:  Timothy Beene, Jason Jennings, Marcus Mirzaie,
#                 Khanh Ngo, and Ninh Nguyen
#
######################################################################

from calibration import *
import communication
from communication import *
import recording
import validation
import Tkinter
from Tkinter import *
import tkMessageBox

# Intermediary Functions


def calibrate_button():
    tkMessageBox.showinfo("Calibrate", "Calibrating...")
    calibrate()
    return

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


def get_thresh_str(sensor):
    return "Thresh:  " + str(SensorThreshold[sensor] * float(ThresholdPercent.get()) / 100)

for i in range(0, 16):
    SensorValue.append(0.0)
    SensorThreshold.append(0.0)
    if i >= 8:
        tmp = str(i+1) + ":  " + str(SensorValue[i]) + " PSI" + "\n" + get_thresh_str(i)
        SensorLabel.append(Label(master, text=tmp))
        SensorLabel[i].grid(row=i-8, column=1)
    else:
        tmp = str(i+1) + ":  " + str(SensorValue[i]) + " PSI" + "\n" + get_thresh_str(i)
        SensorLabel.append(Label(master, text=tmp))
        SensorLabel[i].grid(row=i, column=0)


def update_values():
    for ind, lbl in enumerate(SensorLabel):
        sval = get_sensor_value(ind)
        tmp2 = str(ind+1) + ":  " + str(SensorValue[ind]) + " PSI" + "\n" + get_thresh_str(ind)
        lbl.config(text=tmp2)
master.after(100, update_values)


# Draw Calibrate Button
CalibrateButton = Button(master, text="Calibrate", command=calibrate_button)
CalibrateButton.grid(row=2, column=2)
# CalibrateButton.pack()

# Draw Record Data Checkbox
RecordDataCheckbox = IntVar()
Checkbutton(master, text="Record Data", variable=RecordDataCheckbox).grid(row=3, column=2)

# Draw Filter Checkbox
FilterCheckbox = IntVar()
Checkbutton(master, text="Filter", variable=FilterCheckbox).grid(row=4, column=2)

# Run GUI
master.mainloop()
