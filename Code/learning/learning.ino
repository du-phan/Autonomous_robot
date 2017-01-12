#define W_INIT 0.0
#define NUM_STATES 49
#define NUM_ACTIONS 4

#define DXL_BUS_SERIAL1 1  //Dynamixel on Serial1(USART1)  <-OpenCM9.04
#define ID_SHOULDER 1
#define ID_ELBOW 2
#define DXL_POSITIONS_PER_DEGREE 1024.0/300.0
#define SERVO_NUM_STATES 7
#define DX_SPEED 300
#define PRESENT_POS 54  //address of position in dynamixels

int angles[7] = {90, 60, 30, 0, -30, -60, -90};

Dynamixel Dxl(DXL_BUS_SERIAL1);

static int SEED = 38000;  //Grenoble rpz
static float ALPHA = 0.5; // learning rate parameter
static float BETA = 0.0;  // magnitude of noise added to choice
static float GAMMA = 0.999; // discount factor
static float RANDOM_ACTION_RATE = 0.5;
static float RANDOM_ACTION_DECAY_RATE = .99;

static float qtable[NUM_STATES][NUM_ACTIONS]; //  state-action values

static int first_time = 1;
static int current_action, next_action;
static int current_state, next_state;

void reset_controller()
{
   current_state = next_state = 0;
   current_action = next_action = -1; // "null" action value
}

int get_action(float reward)
{
   if (first_time)
   {
      first_time = 0;
      reset_controller();   // set state and action to null values

      for (int i = 0; i < NUM_STATES; i++)
         for (int j = 0; j < NUM_ACTIONS; j++) // num_actions
            qtable[i][j] = W_INIT;

      
      randomSeed(SEED); // initialize random number generator

      current_action = indexOfMax(qtable[current_state]) + 1;//choose action with the highest Q value

      return current_action;
   }

   float random_number = random(100) / 100.0;

   bool choose_random_action = (1.0 - RANDOM_ACTION_RATE) <= random_number;

   if (choose_random_action)
   {
     next_action = random(4) + 1; //rand from 1~4
   }
   else
   {
     next_action = indexOfMax(qtable[next_state]) + 1;//choose action with the highest Q value
   }

   qtable[current_state][current_action] = (1 - ALPHA) * qtable[current_state][current_action] + ALPHA * (reward + GAMMA * qtable[next_state][next_action]);

   current_state = next_state;
   current_action = next_action;

   return current_action;
}

float get_reward(int current_action)
{
    return 0;
}

int get_next_state(int current_action)
{
    int new_computed_state;

    if (current_action == 1) // (+1,0)
    {
      new_computed_state = current_state + SERVO_NUM_STATES;
    }
    else if (current_action == 2) // (-1,0)
    {
      new_computed_state = current_state - SERVO_NUM_STATES;
    }
    else if (current_action == 3) // (0,+1)
    {
      new_computed_state = current_state + 1;
    }
    else // (0,-1)
    {
      new_computed_state = current_state - 1;
    }

    if ((new_computed_state < 0) || (new_computed_state >= NUM_STATES))
    {
      return current_state;
    }
    else
    {
      return new_computed_state;
    }
}

void q_learning()
{
    current_action = get_action(0.0); // initialize the model

    while(true)
    {
      float reward = get_reward(current_action);
      next_state = get_next_state(current_action);
      current_action = get_action(reward);
    }
}


void setup(){
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
  delay(1000);
}

void moveDxl(int index1, int index2)//move joints to this position
{
  int angle1 = dxlAngle(angles[index1]);
  int angle2 = dxlAngle(angles[index2]);
  Dxl.setPosition(ID_SHOULDER, angle1, DX_SPEED); 
  Dxl.setPosition(ID_ELBOW,    angle2, DX_SPEED);
 
  /*while(not there yet)
  {
    shoulderPos = Dxl.readWord(ID_SHOULDER, PRESENT_POS); // Read present position
    elbowPos = Dxl.readWord(ID_ELBOW, PRESENT_POS); // Read present position
    delay(10);
  }*/
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

void moveMotors(int state)
{
  int elbowIndex = state%SERVO_NUM_STATES;
  int shoulderIndex = (state - elbowIndex)/SERVO_NUM_STATES;
  moveDxl(shoulderIndex,elbowIndex);
}
