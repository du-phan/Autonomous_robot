#include <ctime>
#include <stdlib.h>
#include <stdio.h>

#define sqr(x) ( (x) * (x) )
#define W_INIT 0.0
#define NUM_STATES 49
#define NUM_ACTIONS 4


static int SEED = 38000; //Grenoble rpz
static float ALPHA = 0.5; // learning rate parameter
static float BETA = 0.0; // magnitude of noise added to choice
static float GAMMA = 0.999; // discount factor
static float RANDOM_ACTION_RATE = 0.5;
static float RANDOM_ACTION_DECAY_RATE = .99;
static int RAND_MAX;

static float qtable[NUM_STATES][NUM_ACTIONS]; //  state-action values

static first_time = 1;
static int current_action, next_action;
static int current_state, next_state;

void reset_controller(void)
{
   current_state = next_state = 0;
   next_action = next_action = -1; // "null" action value
}

int get_action(float reward)
{

   if (first_time)
   {
      first_time = 0;
      reset_controller();   // set state and action to null values

      for (i = 0; i < NUM_STATES; i++)
         for (j = 0; j < NUM_ACTIONS; j++) // num_actions
            qtable[i][j] = W_INIT;

      srandom(SEED); // initialize random number generator

      //current_action = qtable[current_state].argsort()[-1];

      return current_action
   }

   float random_number =  (1.0 * random(RAND_MAX) / RAND_MAX);

   bool choose_random_action = (1.0 - RANDOM_ACTION_RATE) <= random_number;

   if (choose_random_action)
   {
        next_action = uni(0, NUM_ACTIONS);
   }
   else
   {
        // next_action = qtable[next_state].argsort()[-1];
   }

   qtable[current_state][current_action] = (1 - ALPHA) * qtable[current_state][current_action] + ALPHA * (reward + GAMMA * qtable[next_state][next_action])

   current_state = next_state;
   current_action = next_action;

   return current_action;
}

float get_reward(current_action)
{
    return;
}

float get_next_state(current_action)
{
    return;
}

void q_learning()
{
    current_action = get_action(0.0); // initialize the model

    while(True):

        float reward = get_reward(current_action);

        next_state = get_next_state(current_action);

        current_action = get_action(reward);

}

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

int indexOfMax(float[4] maxArray)
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
	elbowIndex = state%SERVO_NUM_STATES;
	shoulderIndex = (state - elbowIndex)/SERVO_NUM_STATES;
	moveDxl(shoulderIndex,elbowIndex);
}
