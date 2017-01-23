#define W_INIT 0.0
#define NUM_STATES 49
#define NUM_ACTIONS 4

#define DXL_BUS_SERIAL1 1  //Dynamixel on Serial1(USART1)  <-OpenCM9.04
#define ID_SHOULDER 1
#define ID_ELBOW 2
#define DXL_POSITIONS_PER_DEGREE 1024.0/300.0
#define SERVO_NUM_STATES 7
#define DX_SPEED 100
#define PRESENT_POS 54  //address of position in dynamixels
#define RANDOM_ACTION_DECAY_RATE 0.999;

int angles[7] = {75, 50, 25, 0, -25, -50, -75};
//int angles[7] = {60, 0, -60};

int myI = 0, myJ = 0;
int myCommand = 0;
bool uartFlag = false;

class AS5040
{
 public:
  AS5040 (byte pinCLK, byte pinCS, byte pinDO, byte pinPROG = 0xFF) ;

  boolean begin () ;
  boolean begin (byte mode) ;
  boolean begin (byte mode, boolean reverse, unsigned int offset) ;

  unsigned int read () ; 
  byte status () ;
  boolean valid () ;
  int Zaxis () ;

 private:

  byte _pinCLK ;
  byte _pinCS ;
  byte _pinDO ;
  byte _pinPROG ;

  byte _status ;
  byte _parity ;

  byte even_parity (byte val) ;
} ;

Dynamixel Dxl(DXL_BUS_SERIAL1);  //dynamixel bus
AS5040 encL (16, 17, 18); //encoder

int SEED = 38000;  //Grenoble rpz
float ALPHA = 0.8; // learning rate parameter
float BETA = 0.5;  // magnitude of noise added to choice
float GAMMA = 0.95; // discount factor
float randomActionRate = 1.00;//100% at start

float qTable[NUM_STATES][NUM_ACTIONS]; //  state-action values
float tmpQTable[NUM_STATES][NUM_ACTIONS]; //  state-action values
float rTable[NUM_STATES][NUM_ACTIONS]; //  state-action rewards
float fixedRTable[NUM_STATES][NUM_ACTIONS] = {
{ -250 , -8888, -405 , -8888},{//0
  -309 , -8888, -400 , 405  },{
  -262 , -8888, -255 , 400  },{
  -231 , -8888, -77  , 255  },{//3
  -61  , -8888, 0    , 77   },{
  0    , -8888, 0    , 0    },{//5
  0    , -8888, -8888, 0    },{
  -325 , 250  , -452 , -8888},{//7
  -270 , 309  , -325 , 452  },{
  -200 , 262  , -190 , 325  },{
  -125 , 231  , -10  , 190  },{//10
  -2   , 61   , 0    , 10   },{
  0    , 0    , 0    , 0    },{//12
  0    , 0    , -8888, 0    },{
  -192 , 325  , -390 , -8888},{//14
  -169 , 270  , -285 , -390 },{
  -105 , 200  , -132 , 285  },{
  -10  , 125  , -5   , 132  },{//17
  0    , 2    , 0    , 5    },{
  0    , 0    , 0    , 0    },{//19
  0    , 0    , -8888, 0    },{
  -117 , 192  , -350 , -8888},{//21
  -67  , 169  , -235 , 350  },{
  -8   , 105  , -26  , 235  },{
  0    , 10   , 0    , 26   },{//24
  0    , 0    , 0    , 0    },{
  0    , 0    , 0    , 0    },{
  0    , 0    , -8888, 0    },{
  -38  , 117  , -250 , -8888},{//28
  0    , 67   , -148 , 250  },{
  0    , 8    , -3   , 148  },{//30
  0    , 0    , 0    , 3    },{
  0    , 0    , 0    , 0    },{//32
  0    , 0    , 0    , 0    },{
  0    , 0    , -8888, 0    },{
  0    , 38   , -195 , -8888},{//35
  0    , 0    , -193 , 195  },{
  0    , 0    , -5   , 193  },{//37
  0    , 0    , 0    , 5    },{
  0    , 0    , 0    , 0    },{
  0    , 0    , 0    , 0    },{//40
  0    , 0    , -8888, 0    },{
  -8888, 0    , -255 , -8888},{//42
  -8888, 0    , -190 , 255  },{
  -8888, 0    , -8   , 190  },{
  -8888, 0    , 0    , 8   },{//45
  -8888, 0    , 0    , 0    },{
  -8888, 0    , 0    , 0    },{
  -8888, 0    , -8888, 0    }//48
  }; //  state-action rewards

int first_time = 1;
int current_action, next_action;
int current_state, next_state;

int wheelRot = 0, prevWheelRot = 0;//position of the wheel
int distanceTravelled = 0;

unsigned long iteration = 0;//number of movements made

void initialize_stuff()
{
  for (int i = 0; i < NUM_STATES; i++)
     for (int j = 0; j < NUM_ACTIONS; j++) // num_actions
        {
          qTable[i][j] = W_INIT;
          rTable[i][j] = 0;
        }
  current_state = NUM_STATES/2;
  next_state = NUM_STATES/2;
  current_action = -1;//"null value"
  randomSeed(SEED); // initialize random number generator
}

void update_action()
{
  int next_action;//temp to store next action
  float random_number = (1.0 * random(100)) / 100.0;//random number from 0.0 to 1.0
  bool choose_random_action = (1.0 - randomActionRate) <= random_number;//choose a random action?

  if(!choose_random_action)
  {
    next_action = indexOfMax(qTable[next_state]);//choose action with the highest Q value
    if(actionIsAllowed(current_state,next_action))
    {
      current_action = next_action;
      return;
    }
  }
  
  do
  {
    next_action = random(4); //rand from 0~3
  }while(!actionIsAllowed(current_state,next_action));
  
  current_action = next_action;
}

bool actionIsAllowed(int state, int action)
{
  if ((action == 0) && !(state > (NUM_STATES - SERVO_NUM_STATES - 1 ))) // (+1,0) down
    return true;//allowed
  else if ((action == 1) && !(state < SERVO_NUM_STATES)) // (-1,0) up
    return true;//allowed
  else if ((action == 2) && !((state%SERVO_NUM_STATES) == (SERVO_NUM_STATES - 1)))// (0,+1) right
    return true;//allowed
  else if ((action == 3) && !((state%SERVO_NUM_STATES) == 0)) // (0,-1) left
    return true;//allowed
  
  return false;//not allowed
}

void update_next_state()//done
{
  int next_state_tmp;
  
//  SerialUSB.print("\tcurrent_action = ");
//  SerialUSB.println(current_action);
  if ((current_action == 0) && actionIsAllowed(current_state,current_action)) // (+1,0) down
  {
    next_state_tmp = current_state + SERVO_NUM_STATES;
  }
  else if ((current_action == 1) && actionIsAllowed(current_state,current_action)) // (-1,0) up
  {
    next_state_tmp = current_state - SERVO_NUM_STATES;
  }
  else if ((current_action == 2) && actionIsAllowed(current_state,current_action))// (0,+1) right
  {
    next_state_tmp = current_state + 1;
  }
  else if ((current_action == 3) && actionIsAllowed(current_state,current_action)) // (0,-1) left
  {
    next_state_tmp = current_state - 1;
  }
  else{
    next_state_tmp = current_state;
  }
//  SerialUSB.print("\tnext_state_tmp:");
//  SerialUSB.println(next_state_tmp);
  next_state = next_state_tmp;
}

void moveMotors(int state)
{
  int elbowIndex = state%SERVO_NUM_STATES;
  int shoulderIndex = (state - elbowIndex)/SERVO_NUM_STATES;
  moveDxl(shoulderIndex,elbowIndex);
}

void moveDxl(int index1, int index2)//move joints to this position
{
  int angle1 = dxlAngle(angles[index1]);
  int angle2 = dxlAngle(angles[index2]);
//  SerialUSB.print("angle1:");
//  SerialUSB.print(angles[index1]);
//  SerialUSB.print("\tangle2:");
//  SerialUSB.println(angles[index2]);
  
  Dxl.setPosition(ID_SHOULDER, angle1, DX_SPEED); 
  Dxl.setPosition(ID_ELBOW,    angle2, DX_SPEED);
  
  int shoulderPos = 2000, elbowPos = 2000;//make sure first test in while loop is true
  
  while((abs(shoulderPos - angle1) > 45) || (abs(elbowPos - angle2) > 45))
  {
    shoulderPos = Dxl.readWord(ID_SHOULDER, PRESENT_POS); // Read present position
    elbowPos = Dxl.readWord(ID_ELBOW, PRESENT_POS); // Read present position
    delay(5);
    readEncoder();
  }
  float averageWheelRot = 10;
  while(averageWheelRot > 1.0)
  {
    averageWheelRot = 0.8 * averageWheelRot + 0.2 * wheelRot;
    delay(5);
    readEncoder();//read encoder one last time after motors have reached their final positions
  }
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

int indexOfMax(float maxArray[4])
{
  int index = 0;
  for(int i = 1;i<4;i++)
  {
    if(maxArray[i] > maxArray[index])
    index = i;
  }
  return index;
}

float maxValue(float maxArray[4])
{
  float maxVal = -5000;
  for(int i = 0;i<4;i++)
  {
    float val = maxArray[i];
    if(val > maxVal)
      maxVal = val;
  }
  return maxVal;
}

void readEncoder()//updates wheelRot, +ve values => moving forward
{
  int currentWheelRot = encL.read();
  wheelRot = (currentWheelRot - prevWheelRot);
  prevWheelRot = currentWheelRot;
  if(wheelRot > 512)//corrects 1023 to 0 error
    wheelRot -= 1023;
  if(wheelRot < -512)//corrects 0 to 1023 error
    wheelRot += 1023;
  
  distanceTravelled += wheelRot;
}


void updateR()
{
  float reward = 0.0;
  distanceTravelled = fixedRTable[current_state][current_action];
//  if(distanceTravelled < -50)//if you go back a lot
//    distanceTravelled *= 2;//50% extra penalty
  reward += distanceTravelled;
  
  distanceTravelled = 0;//re-initialize distanceTravelled
  /*if((current_state == 0) && (current_action == 2))
    reward = 100.0;
  else if((current_state == 1) && (current_action == 3))
    reward = -100.0;*/
  
  rTable[current_state][current_action] = (1.0 - BETA) * rTable[current_state][current_action] + BETA * reward;
}


void updateQ()
{
  //calculate values
  for (int i = 0; i < NUM_STATES; i++)
  {
    for (int j = 0; j < NUM_ACTIONS; j++) // num_actions
    {
      if ((j == 0) && !(i > (SERVO_NUM_STATES * (SERVO_NUM_STATES - 1) - 1 ))) // (+1,0) down
      {
        tmpQTable[i][j] = rTable[i][j] + GAMMA * maxValue(qTable[i + SERVO_NUM_STATES]);
      }
      else if ((j == 1) && !(i < SERVO_NUM_STATES)) // (-1,0) up
      {
        tmpQTable[i][j] = rTable[i][j] + GAMMA * maxValue(qTable[i - SERVO_NUM_STATES]);
      }
      else if ((j == 2) && !((i%SERVO_NUM_STATES) == (SERVO_NUM_STATES - 1)))// (0,+1) right
      {
        tmpQTable[i][j] = rTable[i][j] + GAMMA * maxValue(qTable[i + 1]);
      }
      else if ((j == 3) && !((i%SERVO_NUM_STATES) == 0)) // (0,-1) left
      {
        tmpQTable[i][j] = rTable[i][j] + GAMMA * maxValue(qTable[i - 1]);
      }
      else//edge case, do nothing
      {
        tmpQTable[i][j] = -8888;// edge case
      }
    }
  }
  
  //copy table
  for (int i = 0; i < NUM_STATES; i++)
  {
    for (int j = 0; j < NUM_ACTIONS; j++) // num_actions
      qTable[i][j] = tmpQTable[i][j];
  }
}


void printQ()
{
  SerialUSB.print("\nn:");
  SerialUSB.print(iteration);
  for (int i = 0; i < NUM_STATES; i++)
  {
    SerialUSB.print("\ni: ");
    SerialUSB.print(i);
    for (int j = 0; j < NUM_ACTIONS; j++) // num_actions
    {
      SerialUSB.print("    \t");
      SerialUSB.print(j);
      SerialUSB.print(":");
      if(abs(qTable[i][j] + 8888.0) < 0.1)
        SerialUSB.print("    ");
      else
        SerialUSB.print(qTable[i][j]);
    }
  }
}

void printR()
{
  SerialUSB.print("\nn:");
  SerialUSB.print(iteration);
  for (int i = 0; i < NUM_STATES; i++)
  {
    SerialUSB.print("\ni: ");
    SerialUSB.print(i);
    for (int j = 0; j < NUM_ACTIONS; j++) // num_actions
    {
      SerialUSB.print("  \t");
      SerialUSB.print(j);
      SerialUSB.print(":");
      SerialUSB.print(rTable[i][j]);
    }
  }
}


void setup()/////////////////////////////////////////////////////////////
{
  delay(3000);
  SerialUSB.println("\n###\nboop\n###\n");
  SerialUSB.attachInterrupt(usbInterrupt);
  // Initialize the dynamixel bus:
  // Dynamixel 2.0 Baudrate -> 0: 9600, 1: 57600, 2: 115200, 3: 1Mbps
  Dxl.begin(3);
  Dxl.jointMode(ID_SHOULDER); //jointMode() is to use position mode
  Dxl.jointMode(ID_ELBOW); //jointMode() is to use position mode
  encL.begin();     //connect to encoder
  
  /*Dxl.writeByte(1, 26, 5);//P
  Dxl.writeByte(2, 26, 5);//P
  Dxl.writeByte(1, 27, 3);//I
  Dxl.writeByte(2, 27, 3);//I
  Dxl.writeByte(1, 28, 8);//compliance slope
  Dxl.writeByte(1, 29, 8);//compliance slope
  Dxl.writeByte(2, 28, 8);//compliance slope
  Dxl.writeByte(2, 29, 8);//compliance slope*/

  pinMode(BOARD_LED_PIN, OUTPUT);
  digitalWrite(BOARD_LED_PIN, LOW);  delay(100);//flash quickly at start
  digitalWrite(BOARD_LED_PIN, HIGH); delay(100);
  digitalWrite(BOARD_LED_PIN, LOW);  delay(100);
  digitalWrite(BOARD_LED_PIN, HIGH); delay(100);
  digitalWrite(BOARD_LED_PIN, LOW);  delay(100);
  digitalWrite(BOARD_LED_PIN, HIGH); delay(100);
  digitalWrite(BOARD_LED_PIN, LOW);  delay(100);//on
  digitalWrite(BOARD_LED_PIN, HIGH); delay(100);//off
  
  Dxl.goalPosition(ID_SHOULDER, dxlAngle(0));//ID 1 dynamixel moves to position 1023
  Dxl.goalPosition(ID_ELBOW, dxlAngle(0));//ID 1 dynamixel moves to position 1023
  
  initialize_stuff();   // set state and action to null values

  for (int i = 0; i < NUM_STATES; i++)
     for (int j = 0; j < NUM_ACTIONS; j++) // num_actions
        qTable[i][j] = W_INIT;
        
  initialize_stuff();
  current_action = 0;

  //moveMotors(next_state);//move motors into position
  distanceTravelled = 0;//re-initialize distanceTravelled
}

void loop()/////////////////////////////////////////////////////////////
{
  /*
  readEncoder();
  delay(10);
  if(uartFlag)
  {
    SerialUSB.print(".");
    uartFlag = false;
    if(myCommand == 0)
    {
      myJ++;
      if(myJ == 4)
      {
        myJ = 0; myI++;
      }
      
      SerialUSB.print("new myI: ");
      SerialUSB.print(myI);
      SerialUSB.print("\t,myJ: ");
      SerialUSB.println(myJ);
    }
    else if(myCommand == 1)
    {
      current_state = myI;
      current_action = myJ;
      update_next_state();//ok
      SerialUSB.print("m_");
      moveMotors(next_state);//ok
      SerialUSB.println("moved");
    }
    else if(myCommand == 2)//reset
    {
      SerialUSB.print("m_");
      moveMotors(myI);
      SerialUSB.println("moved");
      distanceTravelled = 0;//clear
      SerialUSB.print("dist: ");
      SerialUSB.println(distanceTravelled);
    }
  }*/
  moveMotors(3); delay(100);//3 4 5 12 19 18 17 16 9 10
  moveMotors(4); delay(100);
  moveMotors(5); delay(100);
  moveMotors(12);delay(100);
  moveMotors(19);delay(100);
  moveMotors(18);delay(100);
  moveMotors(17);delay(100);
  moveMotors(16);delay(100);
  moveMotors(9); delay(100);
  moveMotors(10);delay(100);
/*  if(iteration < 2000)
    randomActionRate = 1.0;//for the 1st 30 iterations, always randomly search
  else if(iteration < 4000)
    randomActionRate = 0.95;//for the 1st 30 iterations, always randomly search
    
  if(randomActionRate < 0.15)
  {
    randomActionRate = 0;//stop searching,###later: add a test to see if we actually move, if not, set rand back to 1.0
    digitalWrite(BOARD_LED_PIN, LOW);//led on
  }
  while(iteration > 6000)//stop
  {
    digitalWrite(BOARD_LED_PIN, LOW);  delay(100);//on
    digitalWrite(BOARD_LED_PIN, HIGH); delay(100);//off
    updateQ();//done -- update Q table based on rewards
    printQ();
    SerialUSB.println("\n-------------------------------------------");
  }
  
  randomActionRate *= RANDOM_ACTION_DECAY_RATE;
  
  update_action();//done -- choose next action to take based on current state and Q Table
  update_next_state();//done
  //moveMotors(next_state);//done -- move motors into position
  
  updateR();//done -- update reward gotten from movement
  updateQ();//done -- update Q table based on rewards
  
//  SerialUSB.print("reward:");
//  SerialUSB.print(reward);
  SerialUSB.print("i: ");
  SerialUSB.print(iteration);
//  SerialUSB.print("\tcs: ");
//  SerialUSB.print(current_state);
//  SerialUSB.print("\trr: ");
//  SerialUSB.print(randomActionRate);
  SerialUSB.print("\tns:");
  SerialUSB.println(next_state);
  
  current_state = next_state;
  iteration++;*/
}

void usbInterrupt(byte* buffer, byte nCount)
{
  uartFlag = true;
  if(((char)buffer[0]) == 'q')
    printQ();
  else if(((char)buffer[0]) == 'r')
    printR();
  else if(((char)buffer[0]) == 'i')
    SerialUSB.println(iteration);
  else if(((char)buffer[0]) == 's')
    SerialUSB.println(current_state);
  else if(((char)buffer[0]) == ' ')
    myCommand = 0;
  else if(((char)buffer[0]) == 'x')//go
    myCommand = 1;
  else if(((char)buffer[0]) == 'z')//reset
    myCommand = 2;
  else if(((char)buffer[0]) == 'v')//read
  {
    SerialUSB.print("d: ");
    SerialUSB.println(distanceTravelled);
    uartFlag = false;
  }
  else
  {
    uartFlag = false;
    SerialUSB.print("I: ");
    SerialUSB.print(myI);
    SerialUSB.print("\tJ: ");
    SerialUSB.println(myJ);
  }
}


