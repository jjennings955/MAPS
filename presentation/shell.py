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

# Make Main Window
master = Tk()

# Make Canvas
canvas_width = 203
canvas_height = 203
w = Canvas(master, width=canvas_width, height=canvas_height)
w.pack(side=LEFT)

# Draw Boxes on Canvas
y = int(canvas_height / 4)
x = int(canvas_width / 2)
w.create_rectangle(3, 3, canvas_width, canvas_height,)
w.create_line(x, 3, x, canvas_height)
w.create_line(3, 3, canvas_width, 3)
for i in range(1, 4):
    w.create_line(3, i*y, canvas_width, i*y)

# Draw Calibrate Button
CalibrateButton = Button(master, text="Calibrate", command=calibrate_button, )
CalibrateButton.pack()

# Draw Record Data Checkbox
RecordDataCheckbox = IntVar()
Checkbutton(master, text="Record Data", variable=RecordDataCheckbox).pack()

# Draw Filter Checkbox
FilterCheckbox = IntVar()
Checkbutton(master, text="Filter", variable=FilterCheckbox).pack()

# Run GUI
master.mainloop()
