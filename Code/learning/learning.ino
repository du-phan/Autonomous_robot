#define W_INIT 0.0
#define NUM_STATES 49
#define NUM_ACTIONS 4

#define DXL_BUS_SERIAL1 1  //Dynamixel on Serial1(USART1)  <-OpenCM9.04
#define ID_SHOULDER 1
#define ID_ELBOW 2
#define DXL_POSITIONS_PER_DEGREE 1024.0/300.0
#define SERVO_NUM_STATES 7
#define DX_SPEED 200
#define PRESENT_POS 54  //address of position in dynamixels
#define RANDOM_ACTION_DECAY_RATE 0.996;

int angles[7] = {75, 50, 25, 0, -25, -50, -75};
//int angles[7] = {60, 0, -60};

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
float GAMMA = 0.5; // discount factor
float randomActionRate = 1.00;//100% at start

float qTable[NUM_STATES][NUM_ACTIONS]; //  state-action values
int rTable[NUM_STATES][NUM_ACTIONS]; //  state-action rewards

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
  
  Dxl.setPosition(ID_SHOULDER, angle1, DX_SPEED); 
  Dxl.setPosition(ID_ELBOW,    angle2, DX_SPEED);
  
  int shoulderPos = 2000, elbowPos = 2000;//make sure first test in while loop is true
  
  while((abs(shoulderPos - angle1) > 50) || (abs(elbowPos - angle2) > 50))
  {
    shoulderPos = Dxl.readWord(ID_SHOULDER, PRESENT_POS); // Read present position
    elbowPos = Dxl.readWord(ID_ELBOW, PRESENT_POS); // Read present position
    delay(10);
    readEncoder();
  }
  delay(200);
  readEncoder();//read encoder one last time after motors have reached their final positions
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
  float reward = -1.0;
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
  for (int i = 0; i < NUM_STATES; i++)
  {
    for (int j = 0; j < NUM_ACTIONS; j++) // num_actions
    {
      if ((j == 0) && !(i > (SERVO_NUM_STATES * (SERVO_NUM_STATES - 1) - 1 ))) // (+1,0) down
      {
        qTable[i][j] = rTable[i][j] + GAMMA * maxValue(qTable[i + SERVO_NUM_STATES]);
      }
      else if ((j == 1) && !(i < SERVO_NUM_STATES)) // (-1,0) up
      {
        qTable[i][j] = rTable[i][j] + GAMMA * maxValue(qTable[i - SERVO_NUM_STATES]);
      }
      else if ((j == 2) && !((i%SERVO_NUM_STATES) == (SERVO_NUM_STATES - 1)))// (0,+1) right
      {
        qTable[i][j] = rTable[i][j] + GAMMA * maxValue(qTable[i + 1]);
      }
      else if ((j == 3) && !((i%SERVO_NUM_STATES) == 0)) // (0,-1) left
      {
        qTable[i][j] = rTable[i][j] + GAMMA * maxValue(qTable[i - 1]);
      }
      else//edge case, do nothing
      {
        qTable[i][j] = -888;// edge case
      }
    }
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
      if(abs(qTable[i][j] + 888.0) < 0.1)
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
  Dxl.jointMode(1); //jointMode() is to use position mode
  Dxl.jointMode(2); //jointMode() is to use position mode
  encL.begin();     //connect to encoder

  pinMode(BOARD_LED_PIN, OUTPUT);
  digitalWrite(BOARD_LED_PIN, LOW);  delay(100);//flash quickly at start
  digitalWrite(BOARD_LED_PIN, HIGH); delay(100);
  digitalWrite(BOARD_LED_PIN, LOW);  delay(100);
  digitalWrite(BOARD_LED_PIN, HIGH); delay(100);
  digitalWrite(BOARD_LED_PIN, LOW);  delay(100);
  digitalWrite(BOARD_LED_PIN, HIGH); delay(100);
  digitalWrite(BOARD_LED_PIN, LOW);  delay(100);//on
  digitalWrite(BOARD_LED_PIN, HIGH); delay(100);//off
  
  Dxl.goalPosition(1, dxlAngle(0));//ID 1 dynamixel moves to position 1023
  Dxl.goalPosition(2, dxlAngle(0));//ID 1 dynamixel moves to position 1023
  
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
//  printQ();
  if(iteration < 50)
    randomActionRate = 1.0;//for the 1st 30 iterations, always randomly search
  if(randomActionRate < 0.15)
  {
    randomActionRate = 0;//stop searching,###later: add a test to see if we actually move, if not, set rand back to 1.0
    digitalWrite(BOARD_LED_PIN, LOW);//led on
  }
  while(iteration > 580)
  {
    digitalWrite(BOARD_LED_PIN, LOW);  delay(100);//on
    digitalWrite(BOARD_LED_PIN, HIGH); delay(100);//off
  }
  
  randomActionRate *= RANDOM_ACTION_DECAY_RATE;
  
  update_action();//done -- choose next action to take based on current state and Q Table
  update_next_state();//done
  moveMotors(next_state);//done -- move motors into position
  
  updateR();//done -- update reward gotten from movement
  updateQ();//done -- update Q table based on rewards
  
//  SerialUSB.print("reward:");
//  SerialUSB.print(reward);
//  SerialUSB.print("i: ");
//  SerialUSB.print(iteration);
//  SerialUSB.print("\tcurrent state: ");
//  SerialUSB.print(next_state);
//  SerialUSB.print("\tnext state: ");
//  SerialUSB.print(next_state);
//  SerialUSB.print("\trand rate: ");
//  SerialUSB.println(randomActionRate);
//  SerialUSB.print("\tnext State:");
//  SerialUSB.println(next_state);
  
  current_state = next_state;
  iteration++;
}

void usbInterrupt(byte* buffer, byte nCount)
{
  if((((char)buffer[0]) == 'q') || (((char)buffer[0]) == 'Q'))
    printQ();
  else if((((char)buffer[0]) == 'r') || (((char)buffer[0]) == 'R'))
    printR();
  else
  {
    SerialUSB.print("state: ");
    SerialUSB.print(current_state);
  }
}
