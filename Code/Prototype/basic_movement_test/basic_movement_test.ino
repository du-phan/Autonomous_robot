#define DXL_BUS_SERIAL1 1  //Dynamixel on Serial1(USART1)  <-OpenCM9.04

#define ID_NUM 1
#define DXL_POSITIONS_PER_DEGREE 1024.0/300.0

int angles[7] = {90, 60, 30, 0, -30, -60, -90};

Dynamixel Dxl(DXL_BUS_SERIAL1);

void setup() {
  // Initialize the dynamixel bus:
  // Dynamixel 2.0 Baudrate -> 0: 9600, 1: 57600, 2: 115200, 3: 1Mbps  
  Dxl.begin(3);  
  Dxl.jointMode(1); //jointMode() is to use position mode
  Dxl.jointMode(2); //jointMode() is to use position mode
  
  Dxl.goalPosition(1, dxlAngle(0));//ID 1 dynamixel moves to position 1023
  
  Dxl.goalPosition(2, dxlAngle(0));//ID 1 dynamixel moves to position 1023
}

void loop()
{
  moveDxl(1,3);
  moveDxl(2,3);
  delay(1000);
  
  moveDxl(1,0);
  moveDxl(2,3);
  delay(1000);
  
  moveDxl(1,0);
  moveDxl(2,5);
  delay(1000);
  
  
  moveDxl(1,3);
  moveDxl(2,5);
  delay(1000);
  
  
 /* moveDxl(1,3);
  moveDxl(2,3);
  delay(500);
  
  moveDxl(1,3);
  moveDxl(2,3);
  delay(500);
  
  moveDxl(1,3);
  moveDxl(2,3);
  delay(500);
  
  moveDxl(1,3);
  moveDxl(2,3);
  delay(500);
  
  moveDxl(1,3);
  moveDxl(2,3);
  delay(500);
  
  moveDxl(1,3);
  moveDxl(2,3);
  delay(500);*/
}

void moveDxl(int id, int angleIndex)
{
  Dxl.setPosition(id, dxlAngle(angles[angleIndex]), 300); 
}

int dxlAngle(float angleDEG)//returns the 0-1023 value needed to get this -90° ~ 90° angle
{
  int anglePos = (358.4 + DXL_POSITIONS_PER_DEGREE * angleDEG);
  
  if(anglePos > 1023)//make sure we return a 0-1023 value
    anglePos = 1023;
  else if(anglePos < 0)
    anglePos = 0;
  
  return anglePos;
}
