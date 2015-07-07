
#include "Statistic.h"
int analogPin = 1;
int s0 = PIN_B0;
int s1 = PIN_B1;
int s2 = PIN_B2;
int s3 = PIN_B3;
int E = PIN_B4;

// outside leads to ground and +5V
int val = 0;           // variable to store the value read
int l0 = 0, l1 =0;
float last_value[9];
Statistic myStats[9];
float volts_to_psi(float v)
{
  v = v/1023.0*5.0;
  return (v + 5.0*0.004)/(5.0*0.004)*0.145037738;
}
void setup()
{
  Serial.begin(250000);          //  setup serial
  pinMode(PIN_B0, OUTPUT);
  pinMode(PIN_B1, OUTPUT);
  pinMode(PIN_B2, OUTPUT);
  pinMode(PIN_B3, OUTPUT);
  pinMode(PIN_B4, OUTPUT);
  for (int i=0; i<10; i++) myStats[i].clear();
}

void loop()
{

  setMux(0);
  l0 = analogRead(analogPin);    // read the input pin
  int i;

  for (i=0; i<10; i++){
    setMux(i);
    Serial.printf(" l%d: ", i);
    last_value[i] = volts_to_psi(analogRead(analogPin));
    Serial.print(last_value[i]);
    myStats[i].add(last_value[i]);
  }
  Serial.printf("\n");

  for (i=0; i<10; i++)
  {
    Serial.print(" Hi: ");
    Serial.print(myStats[i].maximum());
  }
  Serial.printf("\n");
  for (i=0; i<10; i++)
  {
    Serial.print(" Lo: ");
    Serial.print(myStats[i].minimum());
  }
  Serial.printf("\n");
  for (i=0; i<10; i++)
  {
    Serial.print(" Av: ");
    Serial.print(myStats[i].average());
  }
  Serial.printf("\n");
  delay(100);
}
void setMux(int channel){
  int controlPin[] = {s0, s1, s2, s3};

  int muxChannel[16][4]={
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
  for(int i = 0; i < 4; i ++){
    digitalWrite(controlPin[i], muxChannel[channel][i]);
  }
  digitalWrite(E,1);
}