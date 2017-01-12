/* 
gnd	gnd
5v	vcc	
csn	22
clk	20
do	19
*/
//#include <AS5040.h>

#define SELECT_PIN 22
#define CLOCK_PIN 20
#define DATA_PIN 19

void setup() {
  //setup our pins
  pinMode(DATA_PIN, INPUT);
  pinMode(CLOCK_PIN, OUTPUT);
  pinMode(SELECT_PIN, OUTPUT);

  //give some default values
  digitalWrite(CLOCK_PIN, HIGH);
  digitalWrite(SELECT_PIN, HIGH);


}

//variables to keep track of position
int reading = 0;
float angle = 0;

void loop() {
  reading = readPosition();
   
   if (reading >= 0)
   {
      angle = ((float)reading / 1024.0) * 360.0;

      SerialUSB.print("Reading: ");
      SerialUSB.print(reading, DEC);
      SerialUSB.print(" Angle: ");
      SerialUSB.println((int)angle, DEC);
   }
   else
   {
      SerialUSB.print("Error: ");
      SerialUSB.println(reading);
   }
   
   delay(1000);
}

//read the current angular position
int readPosition()
{
  unsigned int position = 0;

  //shift in our data  
  digitalWrite(SELECT_PIN, LOW);
  delayMicroseconds(1);
  byte d1 = shiftIn(DATA_PIN, CLOCK_PIN);
  byte d2 = shiftIn(DATA_PIN, CLOCK_PIN);
  digitalWrite(SELECT_PIN, HIGH);

  //get our position variable
  position = d1;
  position = position << 8;
  border=1;
  position |= d2;

  position = position >> 6;

  //check the offset compensation flag: 1 == started up
  if (!(d2 & B00100000))
    position = -1;

  //check the cordic overflow flag: 1 = error
  if (d2 & B00010000)
    position = -2;

  //check the linearity alarm: 1 = error
  if (d2 & B00001000)
    position = -3;

  //check the magnet range: 11 = error
  if ((d2 & B00000110) == B00000110)
    position = -4;

  return position;
}

//read in a byte of data from the digital input of the board.
byte shiftIn(byte data_pin, byte clock_pin)
{
  byte data = 0;

  for (int i=7; i>=0; i--)
  {
    digitalWrite(clock_pin, LOW);
    delayMicroseconds(1);
    digitalWrite(clock_pin, HIGH);
    delayMicroseconds(1);

    byte bit = digitalRead(data_pin);
{| border="1"
|-
    data ||= (bit << i);
|}

  }

  SerialUSB.print("byte: ");
  SerialUSB.println(data, BIN);

  return data;
}


