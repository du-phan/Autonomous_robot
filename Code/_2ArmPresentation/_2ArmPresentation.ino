#define W_INIT 0.0
#define NUM_STATES 49
#define NUM_ACTIONS 4

#define DXL_BUS_SERIAL1 1  //Dynamixel on Serial1(USART1)  <-OpenCM9.04
#define ID_SHOULDER1 1
#define ID_ELBOW1 2
#define ID_SHOULDER2 3
#define ID_ELBOW2 4
#define DXL_POSITIONS_PER_DEGREE 1024.0/300.0
#define SERVO_NUM_STATES 7
#define DX_SPEED 150
#define PRESENT_POS 37  //address of position in dynamixels
#define RANDOM_ACTION_DECAY_RATE 0.998; //~500 cycles to 20%

Dynamixel Dxl(DXL_BUS_SERIAL1);  //dynamixel bus

int angles[7] = {60, 40, 20, 0, -20, -40, -60};
//int angles[7] = {60, 0, -60};
int SEED = 38000;  //Grenoble rpz
int a = 3, b = 3, c = 3, d = 3;
int iteration = 0;

void setup()
{
  // put your setup code here, to run once:
  randomSeed(SEED); // initialize random number generator
  Dxl.begin(3);
  Dxl.jointMode(ID_SHOULDER1); //jointMode() is to use position mode
  Dxl.jointMode(ID_ELBOW1); //jointMode() is to use position mode
  Dxl.jointMode(ID_SHOULDER2); //jointMode() is to use position mode
  Dxl.jointMode(ID_ELBOW2); //jointMode() is to use position mode
  
  
  
  
  Dxl.writeByte(1, 29, 28); //P
  Dxl.writeByte(2, 29, 28); //P
  Dxl.writeByte(1, 28, 8);  //I
  Dxl.writeByte(2, 28, 8);  //I
  Dxl.writeByte(1, 27, 6);  //D
  Dxl.writeByte(2, 27, 6);  //D
  Dxl.writeWord(1, 32, 200);//Speed
  Dxl.writeWord(2, 32, 200);//Speed
  
  Dxl.writeByte(3, 29, 28); //P
  Dxl.writeByte(4, 29, 28); //P
  Dxl.writeByte(3, 28, 8);  //I
  Dxl.writeByte(4, 28, 8);  //I
  Dxl.writeByte(3, 27, 6);  //D
  Dxl.writeByte(4, 27, 6);  //D
  Dxl.writeWord(3, 32, 200);//Speed
  Dxl.writeWord(4, 32, 200);//Speed
  
}

void loop()
{//all random motions
  SerialUSB.print("boop: ");
  SerialUSB.println((int)random(2));
  if(random(2))//50%50
  {
    if(random(2) && (a < 6))//50%50
      a++;
    else if(a > 0)
      a--;
    else if(a < 6)
      a++;
  }
  else
  {
    if(random(2) && (b < 6))//50%50
      b++;
    else if(b > 0)
      b--;
    else if(b < 6)
      b++;
  }
  
  if(random(2))//50%50
  {
    if(random(2) && (c < 6))//50%50
      c++;
    else if(c > 0)
      c--;
    else if(c < 6)
      c++;
  }
  else
  {
    if(random(2) && (d < 6))//50%50
      d++;
    else if(d > 0)
      d--;
    else if(d < 6)
      d++;
  }
  
  moveDxl(1, a, b);
  moveDxl(2, c, d);
  
  if(iteration > 50)
    for(;;)
    {
      //fixed crawling loop
      
    }
  
  iteration++;
}



void moveDxl(int armNum, int index1, int index2)//move joints to this position
{
  int ids = 0, ide = 0;
  if(armNum == 1)
  {
    ids = ID_SHOULDER1;
    ide = ID_ELBOW1;
  }
  else
  {
    ids = ID_SHOULDER2;
    ide = ID_ELBOW2;
  }
  
  int angle1 = dxlAngle(angles[index1]);
  int angle2 = dxlAngle(angles[index2]);
  SerialUSB.print("angle1:");
  SerialUSB.print(angles[index1]);
  SerialUSB.print("\tangle2:");
  SerialUSB.println(angles[index2]);
  
  Dxl.goalPosition(ids, angle1); 
  Dxl.goalPosition(ide,    angle2);
  
  int shoulderPos = 2000, elbowPos = 2000;//make sure first test in while loop is true
  
  while((abs(shoulderPos - angle1) > 40) || (abs(elbowPos - angle2) > 40))
  {
    shoulderPos = Dxl.readWord(ids, PRESENT_POS); // Read present position
    elbowPos = Dxl.readWord(ide, PRESENT_POS); // Read present position
    delay(5);
    Dxl.goalPosition(ids, angle1);//re-write to be sure the motor gets the correct value
    Dxl.goalPosition(ide, angle2);//re-write to be sure the motor gets the correct value
    
    SerialUSB.print("Err S: ");
    SerialUSB.print(shoulderPos - angle1);
    SerialUSB.print("\tErr E: ");
    SerialUSB.println(elbowPos - angle2);
  }
  delay(100);
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


