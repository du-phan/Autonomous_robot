
#include <AS5040.h>//encoder


#define DXL_BUS_SERIAL1 1  //Dynamixel on Serial1(USART1)  <-OpenCM9.04
#define ID_SHOULDER 1
#define ID_ELBOW 2
#define DXL_POSITIONS_PER_DEGREE 1024.0/300.0
#define DX_SPEED 300


//AS5040 encL (CLKpinL, CSpinL, DOpinL) ;

int angles[7] = {90, 60, 30, 0, -30, -60, -90};
int state[2] = {3,3};

int wheelRot = 0;

Dynamixel Dxl(DXL_BUS_SERIAL1);

//////////////////////////////////////////////////////////////////////

void setup() {
  // Initialize the dynamixel bus:
  // Dynamixel 2.0 Baudrate -> 0: 9600, 1: 57600, 2: 115200, 3: 1Mbps  
  Dxl.begin(3);  // Hyper-Mega-Overkill
  Dxl.jointMode(ID_SHOULDER); //jointMode() is to use position mode
  Dxl.jointMode(ID_ELBOW);    //jointMode() is to use position mode
  
  Dxl.goalPosition(ID_SHOULDER, dxlAngle(0));//ID 1 dynamixel moves to position 1023
  Dxl.goalPosition(ID_ELBOW,    dxlAngle(0));//ID 2 dynamixel moves to position 1023
}


void loop()
{
  Serial.begin(115200);
  
  if (!encL.begin())
    Serial.println("You done F*cked up, son.");
  delay(50);
  wheelRot = encL.read();
  
  moveDxl(state[0], state[1]);//default position
  delay(500);
}

void moveDxl(int angle1, int angle2)//move joints to this position
{
  angle1 = dxlAngle(angles[index1]);
  angle2 = dxlAngle(angles[index2]);
  Dxl.setPosition(ID_SHOULDER, angle1, DX_SPEED); 
  Dxl.setPosition(ID_ELBOW,    angle2, DX_SPEED);
 
  /*while(not there yet)
  {
    delay(10);
  }*/
}


int dxlAngle(float angleDEG)//returns the 0-1023 value needed to get this -90° ~ 90° angle
{// can be hardcoded to speed up operation a tiny bit
  int anglePos = (358.4 + DXL_POSITIONS_PER_DEGREE * angleDEG);
  
  if(anglePos > 1023)//make sure we return a 0-1023 value
    anglePos = 1023;
  else if(anglePos < 0)
    anglePos = 0;
  
  return anglePos;
}


void readEncoder()
{
  wheelRot = encL.read() - wheelRot;
  if(wheelRot > 512)//corrects 1023 to 0 error
    wheelRot -= 1023;
  if(wheelRot < -512)//corrects 0 to 1023 error
    wheelRot += 1023;
}



