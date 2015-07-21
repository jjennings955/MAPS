
#include <Time.h>
int analogPin = 1;
int s0 = PIN_B0;
int s1 = PIN_B1;
int s2 = PIN_B2;
int s3 = PIN_B3;
int E = PIN_B4;

// define packet parameters
const byte PACKET_START_BYTE = 0xAA;
const unsigned int PACKET_OVERHEAD_BYTES = 3;
const unsigned int PACKET_MIN_BYTES = PACKET_OVERHEAD_BYTES + 1;
const unsigned int PACKET_MAX_BYTES = 64;

unsigned int val = 0;           // variable to store the value read
float to_psi(float v) {
  v = v/1023.0*5.0;
  return (v + 5.0*0.004)/(5.0*0.004)*0.145037738;
}
// Initial setup
void setup() {
  Serial.begin(250000);          //  Baud rate ignored for USB
  pinMode(PIN_B0, OUTPUT);
  pinMode(PIN_B1, OUTPUT);
  pinMode(PIN_B2, OUTPUT);
  pinMode(PIN_B3, OUTPUT);
  pinMode(PIN_B4, OUTPUT);
}

// The main loop
void loop() {
//  l0 = analogRead(analogPin);    // read the input pin

  for (int i=0; i<10; i++){
    // tells muliplexer to read pin i
//    setMux(i);
//    Serial.printf(" l%d: ", i);
//    Serial.print(to_psi(analogRead(PIN_F0)));
//    Serial.print(analogRead(PIN_F0));
  }
//  Serial.printf("\n");

    setMux(0);
//  Serial.printf("sizeof(): %d, thing: %d\n",sizeof(analogRead(PIN_F0)) , analogRead(PIN_F0));
//  val = analogRead(PIN_F0);
  val = 800;
  byte buf[2];
  buf[0] = (byte) val;
  buf[1] = (byte) val >> 8;
//  Serial.print(val);
//  Serial.print(" :");
//  Serial.print(buf[0]);
//  Serial.print(" ");
//  Serial.print(buf[1]);
//  Serial.print("\n");
//  buf[2] = (byte) val >> 16;
//  buf[3] = (byte) val >> 24;

//  buf[0] = (byte) 1;
//  buf[1] = (byte) 2;
  
  sendPacket(sizeof(buf), buf);
  
  //wait some time in ms
  delay(50);
}

void setMux(int channel) {
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

  //loop through the 4 sig
  for(int i = 0; i < 4; i ++) {
    digitalWrite(controlPin[i], muxChannel[channel][i]);
  }
  digitalWrite(E,1);
}

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
