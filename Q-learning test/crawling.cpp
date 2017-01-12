#include <iostream>
#include <ctime>
#include <stdlib.h>
#include <stdio.h>
#include <random>

std::random_device rd;      // only used once to initialise (seed) engine
std::mt19937 rng(rd());     // random-number engine used (Mersenne-Twister in this case)
std::uniform_int_distribution<int> uni(min,max); // guaranteed unbiased


#define sqr(x) ( (x) * (x) )
#define W_INIT 0.0
#define NUM_STATES 49
#define NUM_ACTIONS 4


static int SEED = 38000;  	//Grenoble rpz
static float ALPHA = 0.5; 	// learning rate parameter
static float BETA = 0.0;  	// magnitude of noise added to choice
static float GAMMA = 0.999; // discount factor
static float RANDOM_ACTION_RATE = 0.5;
static float RANDOM_ACTION_DECAY_RATE = .99;

static float qtable[NUM_STATES][NUM_ACTIONS]; //  state-action values

static first_time = 1;
static int current_action, next_action;
static int current_state, next_state;

void reset_controller(void)
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

      for (i = 0; i < NUM_STATES; i++)
         for (j = 0; j < NUM_ACTIONS; j++) // num_actions
            qtable[i][j] = W_INIT;

      srandom(SEED); // initialize random number generator

      //current_action = qtable[current_state].argsort()[-1];

      return current_action
   }

   float random_number =  ((double) rand() / (RAND_MAX));

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

void moveMotors(int state) //#F#
{
	elbowIndex = state%SERVO_NUM_STATES;
	shoulderIndex = (state - elbowIndex)/SERVO_NUM_STATES;
	moveDxl(shoulderIndex,elbowIndex);
}

void q_learning()
{
    current_action = get_action(0.0); // initialize the model

    while(True):

        float reward = get_reward(current_action);

        next_state = get_next_state(current_action);

        current_action = get_action(reward);

}

