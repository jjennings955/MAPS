import serial

##TODO grab serial data
ser = serial.Serial('/dev/ttyACM0', 19200, timeout=1)
x = ser.read()          # read one byte
s = ser.read(10)        # read up to ten bytes (timeout)
line = ser.readline()   # read a '\n' terminated line
print(line)
ser.close()