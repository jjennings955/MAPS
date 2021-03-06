import struct
import serial

def analog_to_psi(analog):
    """
    Convert an analog value to a PSI value, using formula from data sheet
    :param analog: The analog value (between 0 and 1023)
    :return: The PSI value
    """
    voltage = analog/1023.0*5.0
    return (voltage + 5.0*0.004)/(5.0*0.004)*0.145037738

class Communication(object):
    PACKET_START_BYTE = 0xAA
    PACKET_OVERHEAD_BYTES = 3
    PACKET_MIN_BYTES = PACKET_OVERHEAD_BYTES + 1
    PACKET_MAX_BYTES = 255

    def __init__(self, serial_port='/dev/ttyACM0', baud=115200):
        """
        Create a new serial communication link.
        :param serial_port: The name of the serial port to connect to (Something like '/dev/ttyACM0' on Ubuntu)
        :param baud: The baud rate of the serial connection
        :return: returns nothing
        """
        self.serial_port = serial_port
        self.serial = serial.Serial(self.serial_port, baud, timeout=1)
        self.running = True

    def read_packet(self):
        """
        Read a single packet from the serial connection
        :return: analog_value, pin_id corresponding to a single packet (Ex: (1, 128) is a reading of 128 on pin 1)
        """
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
                    if Communication.validate_packet(packet_size, packet):
                        # rather return pin_id and val
                        pin_id, val = Communication.extract_payload(packet)
                        return val, pin_id

    @classmethod
    def validate_packet(cls, packet_size, packet):
        """
        Check if a packet is valid by checking format and checksum.
        :param packet_size: The size of the packet in bytes
        :param packet: The body of the packet
        :return: True if packet is valid, false if invalid
        """

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

    @classmethod
    def extract_payload(cls, packet):
        """
        Extract the payload from a packet, removing any unwanted header information
        :param packet: The packet to extract values from
        :return: pin_id, PSI_value
        """
        pin_id = struct.unpack('<B', struct.pack('B', packet[2]))[0]
        value = struct.unpack('<h', struct.pack('BB', packet[3], packet[4]))[0]
        return pin_id, analog_to_psi(value)


if __name__ == "__main__":
    comm = Communication('/dev/ttyACM1')
    while True:
        print(comm.read_packet())
