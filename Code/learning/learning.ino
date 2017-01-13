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

int angles[7] = {75, 50, 25, 0, -25, -50, -75};

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
float BETA = 0.0;  // magnitude of noise added to choice
float GAMMA = 0.5; // discount factor
float randomActionRate = 1.00;//100% at start
#define RANDOM_ACTION_DECAY_RATE 0.996;

float qtable[NUM_STATES][NUM_ACTIONS]; //  state-action values

int first_time = 1;
int current_action, next_action;
int current_state, next_state;

int wheelRot = 0, prevWheelRot = 0;//position of the wheel
int distanceTravelled = 0;

unsigned long iteration = 0;//number of movements made

void reset_controller()
{
   current_state = 24;
   next_state = 24;
   current_action = -1;
   next_action = -1; // "null" action value
}

int get_action(float reward)
{
   //SerialUSB.print("  ||  getA1cur:");
   //SerialUSB.print(next_state);
   if (first_time)
   {
      SerialUSB.print("first time\n");
      first_time = 0;
      reset_controller();   // set state and action to null values

      for (int i = 0; i < NUM_STATES; i++)
         for (int j = 0; j < NUM_ACTIONS; j++) // num_actions
            qtable[i][j] = W_INIT;

      randomSeed(SEED); // initialize random number generator

      current_action = indexOfMax(qtable[current_state]) + 1;//choose action with the highest Q value

      return current_action;
   }
   
   if (current_state == next_state) // when the robot choose a bad action
   {
      int proposed_action = current_action;
      while (proposed_action == current_action)
      {
        proposed_action = random(4);
      }
      next_action = proposed_action;
      current_action = next_action;
      return current_action;
   }
   //SerialUSB.print("  ||  getA2cur:");
   //SerialUSB.print(next_state);
   float random_number = (1.0 * random(100)) / 100.0;

   bool choose_random_action = (1.0 - randomActionRate) <= random_number;

   if (choose_random_action)
   {
     next_action = random(4); //rand from 0~3
   }
   else
   {
     next_action = indexOfMax(qtable[next_state]);//choose action with the highest Q value
   }
   //SerialUSB.print("  ||  getA3cur:");
   //SerialUSB.print(next_state);
   
   randomActionRate *= RANDOM_ACTION_DECAY_RATE;
//   SerialUSB.print("  ||  Random action rate:");
//   SerialUSB.print(randomActionRate);
//   SerialUSB.print("qtable cs");
//   SerialUSB.print(current_state);
//   SerialUSB.print("qtable ca");
//   SerialUSB.print(current_action);
//   SerialUSB.print("qtable ns");
//   SerialUSB.print(next_state);
//   SerialUSB.print("qtable na");
//   SerialUSB.print(next_action);
   qtable[current_state][current_action] = (1 - ALPHA) * qtable[current_state][current_action] + ALPHA * reward;
   //SerialUSB.print("  ||  getA4cur:");
   //SerialUSB.print(next_state);
   current_state = next_state;
   current_action = next_action;
   
//   SerialUSB.print("  ||  getA5cur:");
//   SerialUSB.print(next_state);

   return current_action;
}

float get_reward(int current_action)
{
//  SerialUSB.print("\n### REWARD\n dist travelled = ");
//  SerialUSB.print(distanceTravelled);
  float reward = 0;
  
  /*if(abs(distanceTravelled) < 15)
  {
    distanceTravelled = 0;
  }*/
  
  reward = distanceTravelled - 1;//ignore small values? => robot shaking
  distanceTravelled = 0;
  
//  SerialUSB.print("  _  Reward = ");
//  SerialUSB.println(reward);
  return reward;
}

int get_next_state(int current_action)
{
  int next_state_tmp;
  if ((current_action == 0) && !(current_state > (SERVO_NUM_STATES * (SERVO_NUM_STATES - 1) - 1 ))) // (+1,0) down
  {
    next_state_tmp = current_state + SERVO_NUM_STATES;
  }
  else if ((current_action == 1) && !(current_state < SERVO_NUM_STATES)) // (-1,0) up
  {
    next_state_tmp = current_state - SERVO_NUM_STATES;
  }
  else if ((current_action == 2) && !((current_state%SERVO_NUM_STATES) == (SERVO_NUM_STATES - 1)))// (0,+1) right
  {
    next_state_tmp = current_state + 1;
  }
  else if ((current_action == 3) && !((current_state%SERVO_NUM_STATES) == 0)) // (0,-1) left
  {
//      SerialUSB.print("  ||  cur4state:");
//      SerialUSB.print(current_state);
//      SerialUSB.print("  ||  cur4action:");
//      SerialUSB.print(current_action);

    next_state_tmp = current_state - 10;
  }
  else{
    next_state_tmp = current_state;
  }
//    SerialUSB.print("  ||  selected:");
//    SerialUSB.println(next_state_tmp);
  return next_state_tmp;
}

void setup()/////////////////////////////////////////////////////////////
{
  delay(500);
  SerialUSB.println("\n###\nboop\n###\n");
  // Initialize the dynamixel bus:
  // Dynamixel 2.0 Baudrate -> 0: 9600, 1: 57600, 2: 115200, 3: 1Mbps
  Dxl.begin(3);
  Dxl.jointMode(1); //jointMode() is to use position mode
  Dxl.jointMode(2); //jointMode() is to use position mode
  encL.begin();     //connect to encoder

  pinMode(BOARD_LED_PIN, OUTPUT);
  digitalWrite(BOARD_LED_PIN, HIGH); delay(100);//flash quickly at start
  digitalWrite(BOARD_LED_PIN, LOW);  delay(100);
  digitalWrite(BOARD_LED_PIN, HIGH); delay(100);
  digitalWrite(BOARD_LED_PIN, LOW);  delay(100);
  digitalWrite(BOARD_LED_PIN, HIGH); delay(100);
  digitalWrite(BOARD_LED_PIN, LOW);  delay(100);
  digitalWrite(BOARD_LED_PIN, HIGH); delay(100);
  digitalWrite(BOARD_LED_PIN, LOW);  delay(100);
  
  Dxl.goalPosition(1, dxlAngle(0));//ID 1 dynamixel moves to position 1023

  Dxl.goalPosition(2, dxlAngle(0));//ID 1 dynamixel moves to position 1023
  
  //current_action = get_action(0.0);// initialize the model

  reset_controller();   // set state and action to null values

  for (int i = 0; i < NUM_STATES; i++)
     for (int j = 0; j < NUM_ACTIONS; j++) // num_actions
        qtable[i][j] = W_INIT;

      
  randomSeed(SEED); // initialize random number generator

  current_action = indexOfMax(qtable[current_state]);//choose action with the highest Q value

  moveMotors(next_state);//move motors into position
  distanceTravelled = 0;//re-initialize reward stuff
}

void loop()/////////////////////////////////////////////////////////////
{
  if(iteration < 100)
    randomActionRate = 1.0;//for the 1st 100 iterations, always randomly search
  else if(iteration < 200)
    randomActionRate = 0.9;//for the next 100 iterations, mostly randomly search
  
  float reward = get_reward(current_action);
//  SerialUSB.print("reward:");
//  SerialUSB.print(reward);
  next_state = get_next_state(current_action);
//  SerialUSB.print("  ||  current_action:");
//  SerialUSB.print(current_action);
  current_action = get_action(reward);
//  SerialUSB.print("  ||  next State:");
//  SerialUSB.println(next_state);
  moveMotors(next_state);//move motors into position
  SerialUSB.print("i: ");
  SerialUSB.print(iteration);
  SerialUSB.print("\tnext state: ");
  SerialUSB.print(next_state);
  SerialUSB.print("\trand rate: ");
  SerialUSB.println(randomActionRate);
  iteration++;
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
  delay(100);
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
  SerialUSB.print("indx:");
  int index = 0;
  for(int i = 1;i<4;i++)
  {
    SerialUSB.print(" . ");
    if(maxArray[i] > maxArray[index])
    index = i;
  }
  SerialUSB.print("\n");
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

void moveMotors(int state)
{
  int elbowIndex = state%SERVO_NUM_STATES;
  int shoulderIndex = (state - elbowIndex)/SERVO_NUM_STATES;
  moveDxl(shoulderIndex,elbowIndex);
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

void propagateQvalues()
{
  float nearbyValues[4];
  for (int i = 0; i < NUM_STATES; i++)
  {
    for (int j = 0; j < NUM_ACTIONS; j++) // num_actions
    {
      if ((j == 0) && !(i > (SERVO_NUM_STATES * (SERVO_NUM_STATES - 1) - 1 ))) // (+1,0) down
      {
        qtable[i][j] = (1 - GAMMA) * qtable[i][j] + GAMMA * maxValue(qtable[i + SERVO_NUM_STATES]);
      }
      else if ((j == 1) && !(i < SERVO_NUM_STATES)) // (-1,0) up
      {
        qtable[i][j] = (1 - GAMMA) * qtable[i][j] + GAMMA * maxValue(qtable[i - SERVO_NUM_STATES]);
      }
      else if ((j == 2) && !((i%SERVO_NUM_STATES) == (SERVO_NUM_STATES - 1)))// (0,+1) right
      {
        qtable[i][j] = (1 - GAMMA) * qtable[i][j] + GAMMA * maxValue(qtable[i + 1]);
      }
      else if ((j == 3) && !((i%SERVO_NUM_STATES) == 0)) // (0,-1) left
      {
        qtable[i][j] = (1 - GAMMA) * qtable[i][j] + GAMMA * maxValue(qtable[i - 1]);
      }
      else//edge case, do nothing
      {
        ;//qtable[i][j]
      }
  
    }
  }
}


