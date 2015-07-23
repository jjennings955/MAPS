import struct
import serial
# define packet parameters
PACKET_START_BYTE = 0xAA
PACKET_OVERHEAD_BYTES = 3
PACKET_MIN_BYTES = PACKET_OVERHEAD_BYTES + 1
PACKET_MAX_BYTES = 255

def analog_to_psi(analog):
    voltage = analog/1023.0*5.0
    return (voltage + 5.0*0.004)/(5.0*0.004)*0.145037738


def validate_packet(packet_size, packet):
    """will validate the given packet to ensure is is not corrupted."""
    # check if first is start byte
    if packet[0] != PACKET_START_BYTE:
        return False

    # check if crc is valid
    for i in range(0, packet_size):
        crc = crc ^ packet[i]
    if crc != packet[packet_size-1]:
        return False

    # if each condition is passed return true
    return True


def extract_payload(packet):
    # packet_size = packet_size - PACKET_OVERHEAD_BYTES
    # B is unsigned char
    pin_id = struct.unpack('<B', struct.pack('B', packet[2]))[0]
    # h is short (2 byte int)
    value = struct.unpack('<h', struct.pack('BB', packet[3], packet[4]))[0]
    # print(packet)
    # print(pin_id)
    # print(value)
    return pin_id, analog_to_psi(value)

# def bytes_to_int(bytes):
#     # return int(bytes.encode('hex'), 16)
#     # print(struct.pack('BB', bytes[0], bytes[1]))
#     # B is unsigned char
#     print(bytes)
#     temp = struct.pack('BB', bytes[0], bytes[1])
#     # h is short (2 byte int)
#     merged = struct.unpack('<h', temp)
#     # print(merged)
#     return merged

def readSerial():
    """Will read the Serial and look for packets according to spec."""
    #lab's ubuntu
    # ser = serial.Serial('/dev/ttyACM0', 19200, timeout=1)
    #Timothy's windows
    ser = serial.Serial('com3', 250000, timeout=1)
    is_running = True
    packet = []
    count = 0
    packet_size = PACKET_MIN_BYTES
    # continuously check for received packets
    while is_running:
        # check to see if serial byte is available
        if ser.isOpen():
            # get the byte
            b = ord(ser.read())
            # handle the byte according to the current count
            if count == 0 and b == PACKET_START_BYTE:
                packet.append(b)
                count += 1
            elif count == 0:
                # ignore and look at next byte for PACKET_START_BYTE
                pass
            elif count == 1:
                packet.append(b)
                packet_size = b
                if packet_size < PACKET_MIN_BYTES or packet_size > PACKET_MAX_BYTES:
                    count = 0
                else:
                    packet_size = b
                    count += 1
            elif count < packet_size:
                packet.append(b)
                count += 1
            if count >= packet_size:
                if validate_packet(packet_size, packet):
                    # rather return pin_id and val
                    pin_id, val = extract_payload(packet)
                    return val, pin_id

                count = 0
                packet = []
# for testing
while True:
    print(readSerial())
