import struct
import serial
# define packet parameters
PACKET_START_BYTE = 0xAA
PACKET_OVERHEAD_BYTES = 3
PACKET_MIN_BYTES = PACKET_OVERHEAD_BYTES + 1
PACKET_MAX_BYTES = 255

##TODO grab serial data
# change for system's usb port name
# ser = serial.Serial('/dev/ttyACM0', 19200, timeout=1)
# ser = serial.Serial('com3', 19200, timeout=1)
# x = ser.read()          # read one byte
# s = ser.read(10)        # read up to ten bytes (timeout)
# while 1:
#     line = ser.readline()   # read a '\n' terminated line
#     print(line)
# ser.close()




def readSerial():
    """Will read the Serial and look for packets according to spec."""

    # TODO write validate packet code
    def validatePacket( packetSize, buffer):
        """will validate the given packet to ensure is is not corrupted."""
        return True
    def printPayload(packetSize, payload):
        payloadSize = packetSize - PACKET_OVERHEAD_BYTES
        value = bytes_to_int(payload[2:4])
        print(payload)
        print(value)

    def bytes_to_int(bytes):
        # return int(bytes.encode('hex'), 16)
        # print(struct.pack('BB', bytes[0], bytes[1]))
        # B is unsigned char
        print(bytes)
        temp = struct.pack('BB', bytes[0], bytes[1])
        # h is short (2 byte int)
        merged = struct.unpack('<h', temp)
        # print(merged)
        return merged
    #lab's ubuntu
    # ser = serial.Serial('/dev/ttyACM0', 19200, timeout=1)
    #Timothy's windows
    ser = serial.Serial('com3', 19200, timeout=1)
    isRunning = True
    buffer = []
    count = 0
    packetSize = PACKET_MIN_BYTES
    # continuously check for received packets
    while isRunning:
        # check to see if serial byte is available
        if ser.isOpen():
            # get the byte
            b = ord(ser.read())
            # handle the byte according to the current count
            if count == 0 and b == PACKET_START_BYTE:
                buffer.append(b)
                count += 1
            elif count == 0:
                #ignore and look at next byte for PACKET_START_BYTE
                pass
            elif count == 1:
                buffer.append(b)
                packetSize = b
                if packetSize < PACKET_MIN_BYTES or packetSize > PACKET_MAX_BYTES:
                    count = 0
                else:
                    packetSize = b
                    count += 1
            elif count < packetSize:
                buffer.append(b)
                count += 1
            if count >= packetSize:
                if validatePacket(packetSize, buffer):
                    #TODO pass buffer on to aggregation
                    printPayload(packetSize, buffer)
                count = 0
                buffer = []

readSerial()