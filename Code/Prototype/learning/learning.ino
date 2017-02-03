#define W_INIT 0.0
#define NUM_STATES 49
#define NUM_ACTIONS 4

#define DXL_BUS_SERIAL1 1  //Dynamixel on Serial1(USART1)  <-OpenCM9.04
#define ID_SHOULDER 1
#define ID_ELBOW 2
#define DXL_POSITIONS_PER_DEGREE 1024.0/300.0
#define SERVO_NUM_STATES 7
#define DX_SPEED 150
#define PRESENT_POS 37  //address of position in dynamixels
#define RANDOM_ACTION_DECAY_RATE 0.996; //~500 cycles to 20%

int angles[7] = {75, 50, 25, 0, -25, -50, -75};
//int angles[3] = {60, 0, -60};
//int angles[4] = {60, 20, -20, -60};

int myI = 0, myJ = 0;
int myCommand = 0;

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
float ALPHA = 0.75; // learning rate parameter
float BETA = 0.5;  // magnitude of noise added to choice
float GAMMA = 0.98;// discount factor
float randomActionRate = 1.0;//20% at start

float qTable[NUM_STATES][NUM_ACTIONS]; //  state-action values
float tmpQTable[NUM_STATES][NUM_ACTIONS]; //  state-action values
float rTable[NUM_STATES][NUM_ACTIONS]; //  state-action rewards
/*float fixedRTable[NUM_STATES][NUM_ACTIONS] = {
  { -53.58, -15000.00, -55.69, -15000.00},{
-102.71, -15000.00, -137.24, -56.00},{
-249.45, -15000.00, -104.81, -10.52},{
-150.65, -15000.00, -155.06, -48.03},{
-115.50, -15000.00, -94.09, -17.50},{
-49.36, -15000.00, -49.75, -38.50},{
-49.67, -15000.00, -15000.00, -48.41},{
-75.93, -72.54, -120.48, -15000.00},{
-206.81, -61.68, -262.62, 16.87},{
-97.01, 11.50, -186.00, 42.44},{
-116.87, -38.70, -138.12, -8.69},{
-49.06, -24.00, -48.47, -52.44},{
-49.67, -49.06, -50.39, -49.42},{
-49.45, -49.32, -15000.00, -49.31},{
-123.69, -32.73, -197.13, -15000.00},{
-98.63, 14.13, -181.94, 12.65},{
-122.91, -26.89, -164.83, -1.38},{
-51.13, -44.75, -52.00, -18.09},{
-49.11, -48.88, -49.42, -48.28},{
-49.70, -48.47, -50.41, -49.27},{
-49.84, -49.38, -15000.00, -48.63},{
-120.19, -6.80, -176.50, -15000.00},{
-108.56, -24.17, -126.37, 11.30},{
-83.50, -29.94, -87.25, -14.00},{
-48.81, -47.50, -48.86, -39.50},{
-49.19, -49.00, -49.05, -50.14},{
-49.17, -48.25, -49.05, -48.63},{
-49.61, -48.97, -15000.00, -48.97},{
-52.05, -18.50, -129.87, -15000.00},{
-54.25, -41.54, -105.03, -36.00},{
-65.04, -44.84, -57.06, -7.37},{
-49.00, -50.03, -49.38, -51.47},{
-49.05, -49.84, -49.20, -48.97},{
-49.78, -49.38, -48.56, -48.64},{
-49.09, -48.56, -15000.00, -49.34},{
-69.99, -44.00, -102.22, -15000.00},{
-63.31, -52.21, -127.84, -43.69},{
-61.25, -53.63, -79.23, -5.94},{
-50.50, -49.98, -49.70, -47.63},{
-50.23, -48.84, -49.49, -49.23},{
-49.61, -48.44, -49.41, -49.21},{
-50.20, -49.38, -15000.00, -48.93},{
-15000.00, -53.22, -49.67, -15000.00},{
-15000.00, -49.71, -78.61, -61.50},{
-15000.00, -57.63, -72.00, -18.89},{
-15000.00, -48.41, -49.52, -49.28},{
-15000.00, -48.78, -48.67, -49.22},{
-15000.00, -49.72, -49.03, -50.09},{
-15000.00, -48.73, -15000.00, -49.25}
*/
/*
{ -250 , -15000, -405 , -15000},{//0
  -309 , -15000, -400 , 405  },{
  -262 , -15000, -255 , 400  },{
  -231 , -15000, -77  , 255  },{//3
  -61  , -15000, 0    , 77   },{
  0    , -15000, 0    , 0    },{//5
  0    , -15000, -15000, 0    },{
  -325 , 250  , -452 , -15000},{//7
  -270 , 309  , -325 , 452  },{
  -200 , 262  , -190 , 325  },{
  -125 , 231  , -10  , 190  },{//10
  -2   , 61   , 0    , 10   },{
  0    , 0    , 0    , 0    },{//12
  0    , 0    , -15000, 0    },{
  -192 , 325  , -390 , -15000},{//14
  -169 , 270  , -285 , -390 },{
  -105 , 200  , -132 , 285  },{
  -10  , 125  , -5   , 132  },{//17
  0    , 2    , 0    , 5    },{
  0    , 0    , 0    , 0    },{//19
  0    , 0    , -15000, 0    },{
  -117 , 192  , -350 , -15000},{//21
  -67  , 169  , -235 , 350  },{
  -8   , 105  , -26  , 235  },{
  0    , 10   , 0    , 26   },{//24
  0    , 0    , 0    , 0    },{
  0    , 0    , 0    , 0    },{
  0    , 0    , -15000, 0    },{
  -38  , 117  , -250 , -15000},{//28
  0    , 67   , -148 , 250  },{
  0    , 8    , -3   , 148  },{//30
  0    , 0    , 0    , 3    },{
  0    , 0    , 0    , 0    },{//32
  0    , 0    , 0    , 0    },{
  0    , 0    , -15000, 0    },{
  0    , 38   , -195 , -15000},{//35
  0    , 0    , -193 , 195  },{
  0    , 0    , -5   , 193  },{//37
  0    , 0    , 0    , 5    },{
  0    , 0    , 0    , 0    },{
  0    , 0    , 0    , 0    },{//40
  0    , 0    , -15000, 0    },{
  -15000, 0    , -255 , -15000},{//42
  -15000, 0    , -190 , 255  },{
  -15000, 0    , -8   , 190  },{
  -15000, 0    , 0    , 8   },{//45
  -15000, 0    , 0    , 0    },{
  -15000, 0    , 0    , 0    },{
  -15000, 0    , -15000, 0    }//48
  }; //  state-action rewards*/

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
          if(actionIsAllowed(i,j))
            rTable[i][j] = 0;
          else
            rTable[i][j] = -15000;
        }
  current_state = 10;//NUM_STATES/2;
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
  SerialUSB.print("angle1:");
  SerialUSB.print(angles[index1]);
  SerialUSB.print("\tangle2:");
  SerialUSB.println(angles[index2]);
  
  Dxl.goalPosition(ID_SHOULDER, angle1); 
  Dxl.goalPosition(ID_ELBOW,    angle2);
  
  int shoulderPos = 2000, elbowPos = 2000;//make sure first test in while loop is true
  
  while((abs(shoulderPos - angle1) > 30) || (abs(elbowPos - angle2) > 30))
  {
    shoulderPos = Dxl.readWord(ID_SHOULDER, PRESENT_POS); // Read present position
    elbowPos = Dxl.readWord(ID_ELBOW, PRESENT_POS); // Read present position
    delay(5);
    Dxl.goalPosition(ID_SHOULDER, angle1);//re-write to be sure the motor gets the correct value
    Dxl.goalPosition(ID_ELBOW,    angle2);//re-write to be sure the motor gets the correct value
    readEncoder();
    SerialUSB.print("Err S: ");
    SerialUSB.print(shoulderPos - angle1);
    SerialUSB.print("\tErr E: ");
    SerialUSB.println(elbowPos - angle2);
  }
  
  float averageWheelRot = 2*wheelRot;
  while(averageWheelRot > 1.0)
  {
    delay(50);
    readEncoder();//read encoder one last time after motors have reached their final positions
    averageWheelRot = 0.8 * averageWheelRot + 0.2 * wheelRot;
    SerialUSB.println("avg: ");
    SerialUSB.println(averageWheelRot);
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
  float reward = -50.0;
  
//  distanceTravelled = fixedRTable[current_state][current_action];//for simulation
  
  reward += distanceTravelled;
  distanceTravelled = 0;//re-initialize distanceTravelled
  
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
        tmpQTable[i][j] = -15000;// edge case
      }
    }
  }
  
  //copy table
  for (int i = 0; i < NUM_STATES; i++)
  {
    for (int j = 0; j < NUM_ACTIONS; j++) // num_actions
      qTable[i][j] = (1-ALPHA) * qTable[i][j] + ALPHA * tmpQTable[i][j];
  }
}


void printQ()//print Q table
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
      if(abs(qTable[i][j] + 15000.0) < 0.1)
        SerialUSB.print("    ");
      else
        SerialUSB.print(qTable[i][j]);
    }
  }
}

void printR()//print R table
{
  SerialUSB.print("\nn:");
  SerialUSB.println(iteration);
  SerialUSB.print("{ ");
  for (int i = 0; i < NUM_STATES; i++)
  {
    for (int j = 0; j < (NUM_ACTIONS-1); j++) // num_actions
    {
      SerialUSB.print(rTable[i][j]);
      SerialUSB.print(", ");
    }
    SerialUSB.print(rTable[i][NUM_ACTIONS-1]);//last one
    SerialUSB.println("},{");
  }
  SerialUSB.println("___________________");
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
  
  Dxl.writeByte(1, 29, 28); //P
  Dxl.writeByte(2, 29, 28); //P
  Dxl.writeByte(1, 28, 8);  //I
  Dxl.writeByte(2, 28, 8);  //I
  Dxl.writeByte(1, 27, 6);  //D
  Dxl.writeByte(2, 27, 6);  //D
  Dxl.writeWord(1, 32, 200);//Speed
  Dxl.writeWord(2, 32, 200);//Speed
  

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

  distanceTravelled = 0;//re-initialize distanceTravelled
}

void loop()/////////////////////////////////////////////////////////////
{
  randomActionRate -= 0.001;// *= RANDOM_ACTION_DECAY_RATE;
  if(iteration < 200)//keep random rate high at start
    randomActionRate = 0.95;//for the 1st 80 iterations, mainly randomly search
  
  if(randomActionRate < 0.05)
  {
    randomActionRate = 0;//stop searching,###later: add a test to see if we actually move, if not, set rand back to 1.0
    digitalWrite(BOARD_LED_PIN, LOW);//led on
  }
  
  while(iteration > 1500)//stop
  {
    digitalWrite(BOARD_LED_PIN, LOW);  delay(100);//on
    digitalWrite(BOARD_LED_PIN, HIGH); delay(100);//off
    updateQ();//update Q table based on rewards

    if(myCommand == 888)//simulate the robot moving
    {
      while(1)
      {
        randomActionRate *= RANDOM_ACTION_DECAY_RATE;
        
        update_action();//choose next action to take based on current state and Q Table
        update_next_state();//done
        
        updateQ();//update Q table based on rewards
        current_state = next_state;
        iteration++;
        
        SerialUSB.print("loop: ");
        SerialUSB.print(iteration);
        SerialUSB.print("\trr: ");
        SerialUSB.print(randomActionRate);
        SerialUSB.print("\tns:");
        SerialUSB.println(next_state);
      }
    }
  }
  
  if(myCommand == 888)//simulate the robot moving
  {
    while(1)
    {
      randomActionRate *= RANDOM_ACTION_DECAY_RATE;
      
      update_action();//done -- choose next action to take based on current state and Q Table
      update_next_state();//done
      
      updateQ();//done -- update Q table based on rewards
      current_state = next_state;
      iteration++;
      
      SerialUSB.print("loop: ");
      SerialUSB.print(iteration);
      SerialUSB.print("\trr: ");
      SerialUSB.print(randomActionRate);
      SerialUSB.print("\tns:");
      SerialUSB.println(next_state);
    }
  }
  
//  moveMotors(24);
//  moveMotors(17);
//  moveMotors(10);
//  moveMotors(3);
//  moveMotors(4);
//  moveMotors(5);
//  moveMotors(12);
//  moveMotors(19);
//  moveMotors(26);
//  moveMotors(25);
  
  
  update_action();//done -- choose next action to take based on current state and Q Table
  update_next_state();//done
  moveMotors(next_state);//done -- move motors into position
  
  updateR();//done -- update reward gotten from movement
  updateQ();//done -- update Q table based on rewards
  
  SerialUSB.print("i: ");
  SerialUSB.print(iteration);
  SerialUSB.print("\trr: ");
  SerialUSB.print(randomActionRate);
  SerialUSB.print("\tns:");
  SerialUSB.println(next_state);
  
  current_state = next_state;
  iteration++;
}

void usbInterrupt(byte* buffer, byte nCount)
{
  if(((char)buffer[0]) == 'q')
    printQ();
  else if(((char)buffer[0]) == 'r')
    printR();
  else if(((char)buffer[0]) == 'i')
    SerialUSB.println(iteration);
  else if(((char)buffer[0]) == 's')
    SerialUSB.println(current_state);
  else if(((char)buffer[0]) == 'l')
    myCommand = 888;
  else
  {
    SerialUSB.print("I: ");
    SerialUSB.print(myI);
    SerialUSB.print("\tJ: ");
    SerialUSB.println(myJ);
  }
}


