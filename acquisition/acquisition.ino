
#include <Time.h>
int inputPin = PIN_F1;
int s0 = PIN_B0;
int s1 = PIN_B1;
int s2 = PIN_B2;
int s3 = PIN_B3;
int E = PIN_B4;

// define packet parameters
const int BAUD = 250000;
const byte PACKET_START_BYTE = 0xAA;
const unsigned int PACKET_OVERHEAD_BYTES = 3;
const unsigned int PACKET_MIN_BYTES = PACKET_OVERHEAD_BYTES + 1;
const unsigned int PACKET_MAX_BYTES = 64;
const int NUM_SENSORS = 16;

int controlPin[] = {s0, s1, s2, s3};

int muxChannel[16][4]= {
  {0,0,0,0}, //channel 0
  {1,0,0,0}, //channel 1
  {0,1,0,0}, //channel 2
  {1,1,0,0}, //channel 3
  {0,0,1,0}, //channel 4
  {1,0,1,0}, //channel 5
  {0,1,1,0}, //channel 6
  {1,1,1,0}, //channel 7
  {0,0,0,1}, //channel 8
  {1,0,0,1}, //channel 9
  {0,1,0,1}, //channel 10
  {1,1,0,1}, //channel 11
  {0,0,1,1}, //channel 12
  {1,0,1,1}, //channel 13
  {0,1,1,1}, //channel 14
  {1,1,1,1}  //channel 15
};


unsigned int val = 0;           // variable to store the value read
float to_psi(float v) {
  v = v/1023.0*5.0;
  return (v + 5.0*0.004)/(5.0*0.004)*0.145037738;
}

// sets the digital pins to the multiplexer
void setMux(int channel) {
  //loop through the 4 sig
  for(int i = 0; i < 4; i ++) {
    digitalWrite(controlPin[i], muxChannel[channel][i]);
  }
  digitalWrite(E,1);
}
//sends a packet
boolean sendPacket(int payloadSize, byte *payload)
{
  // check for max payload size
  unsigned int packetSize = payloadSize + PACKET_OVERHEAD_BYTES;
  if(packetSize > PACKET_MAX_BYTES) {
    return false;
  }

  // create the serial packet transmit buffer
  static byte packet[PACKET_MAX_BYTES];

  // populate the overhead fields
  packet[0] = PACKET_START_BYTE;
  packet[1] = packetSize;
  byte checkSum = packet[0] ^ packet[1];

  // populate the packet payload while computing the checksum
  for(int i = 0; i < payloadSize; i++) {
    packet[i + 2] = payload[i];
    checkSum = checkSum ^ packet[i + 2];
  }

  // store the checksum
  packet[packetSize - 1] = checkSum;

  // send the packet
  Serial.write(packet, packetSize);
  Serial.flush();
  return true;
}
// Initial setup
void setup() {
  Serial.begin(BAUD);          //  Baud rate ignored for USB
  pinMode(s0, OUTPUT);
  pinMode(s1, OUTPUT);
  pinMode(s2, OUTPUT);
  pinMode(s3, OUTPUT);
  pinMode(E, OUTPUT);
}

// The main loop
void loop() {
//for each sensor send a packet to the PC
  for (int i=0; i< NUM_SENSORS; i++){
    // tells muliplexer to data for channel i
    setMux(i);
    val = analogRead(inputPin);

    //size of payload 
    byte payload[3];
    payload[0] = (byte) (i);
    payload[1] = (byte) val;
    payload[2] = (byte) (val >> 8); // shift 8 bits to the right

    //sendPacket adds header and footer then sends via binary USB
    sendPacket(sizeof(payload), payload);
    
    //needs some delay otherwise the readings are nonsense
    delay(50);
  }
}


