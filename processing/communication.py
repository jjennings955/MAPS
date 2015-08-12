import struct
import serial
# define packet parameters


def analog_to_psi(analog):
    voltage = analog/1023.0*5.0
    return (voltage + 5.0*0.004)/(5.0*0.004)*0.145037738

class Communication(object):
    PACKET_START_BYTE = 0xAA
    PACKET_OVERHEAD_BYTES = 3
    PACKET_MIN_BYTES = PACKET_OVERHEAD_BYTES + 1
    PACKET_MAX_BYTES = 255

    def __init__(self, serial_port='/dev/ttyACM0', baud=19200):
        self.serial_port = serial_port
        self.serial = serial.Serial(self.serial_port, baud, timeout=1)
        self.running = True

    def read_packet(self):
        packet = []
        count = 0
        packet_size = Communication.PACKET_MIN_BYTES
        # continuously check for received packets
        while self.running:
            # check to see if serial byte is available
            if self.serial.isOpen():
                # get the byte
                b = ord(self.serial.read())
                # handle the byte according to the current count
                if count == 0 and b == Communication.PACKET_START_BYTE:
                    packet.append(b)
                    count += 1
                elif count == 0:
                    # ignore and look at next byte for PACKET_START_BYTE
                    pass
                elif count == 1:
                    packet.append(b)
                    packet_size = b
                    if packet_size < Communication.PACKET_MIN_BYTES or packet_size > Communication.PACKET_MAX_BYTES:
                        count = 0
                    else:
                        packet_size = b
                        count += 1
                elif count < packet_size:
                    packet.append(b)
                    count += 1
                if count >= packet_size:
                    if self.validate_packet(packet_size, packet):
                        # rather return pin_id and val
                        pin_id, val = self.extract_payload(packet)
                        return val, pin_id

    def validate_packet(self, packet_size, packet):
        """will validate the given packet to ensure is is not corrupted."""
        # check if first is start byte
        if packet[0] != Communication.PACKET_START_BYTE:
            print("failed PACKET_START_BYTE")
            return False

        # check if crc is valid
        crc = packet[0] ^ packet[1]
        for i in range(2, packet_size-1):
            crc = crc ^ packet[i]
        if crc != packet[packet_size-1]:
            print("failed CRC")
            print(crc)
            return False

        # if each condition is passed return true
        return True

    def extract_payload(self, packet):
        pin_id = struct.unpack('<B', struct.pack('B', packet[2]))[0]
        value = struct.unpack('<h', struct.pack('BB', packet[3], packet[4]))[0]
        return pin_id, analog_to_psi(value)


if __name__ == "__main__":
    comm = Communication('/dev/ttyACM1')
    while True:
        print(comm.read_packet())
