/*============================================================================*/
/*                                                                            */
/*                                                                            */
/*                    RR1 rev.2 Firmware [for Arduino DUE]                    */
/*                                                                            */
/*                  (C) Copyright 2023 - 2025 Pavel Surynek                   */
/*                                                                            */
/*                http://www.surynek.net | <pavel@surynek.net>                */
/*       http://users.fit.cvut.cz/surynek | <pavel.surynek@fit.cvut.cz>       */
/*                                                                            */
/*============================================================================*/
/* Version 0.2                                                                */
/*----------------------------------------------------------------------------*/

#define RR1_SERIAL_NUMBER    "0002" // RR1 robotic arm serial number

//#define ENCODE_OPTIMIZE_INTERRUPTS

#include <Encoder.h>

const char RR_robot_header[] = "RR1-rev.2-robot";
const int interactive_joint_step_limit = 1024;

enum InteractiveStepperSafety
{
  INTERACTIVE_STEPPER_SAFETY_UNDEFINED = 0,
  INTERACTIVE_STEPPER_SAFETY_HIGH = 1,
  INTERACTIVE_STEPPER_SAFETY_LOW = 2
};

const unsigned int J_S1_LIMITER_A_MASK = 0b00000000000000000000000000000001;
const unsigned int J_S1_LIMITER_B_MASK = 0b00000000000000000000000000000010;

const unsigned int J_S2_LIMITER_A_MASK = 0b00000000000000000000000000000100;
const unsigned int J_S2_LIMITER_B_MASK = 0b00000000000000000000000000001000;

const unsigned int J_E1_LIMITER_A_MASK = 0b00000000000000000000000000010000;
const unsigned int J_E1_LIMITER_B_MASK = 0b00000000000000000000000000100000;

const unsigned int J_E2_LIMITER_A_MASK = 0b00000000000000000000000001000000;
const unsigned int J_E2_LIMITER_B_MASK = 0b00000000000000000000000010000000;

const unsigned int J_W1_LIMITER_A_MASK = 0b00000000000000000000000100000000;
const unsigned int J_W1_LIMITER_B_MASK = 0b00000000000000000000001000000000;

const unsigned int J_W2_LIMITER_A_MASK = 0b00000000000000000000010000000000;
const unsigned int J_W2_LIMITER_B_MASK = 0b00000000000000000000100000000000;

const unsigned int J_G_LIMITER_A_MASK  = 0b00000000000000000001000000000000;
const unsigned int J_G_LIMITER_B_MASK  = 0b00000000000000000010000000000000;

unsigned int joints_limiters_state = 0;

/*----------------------------------------------------------------------------*/


// Arduino DUE interrupt (any digi)
const int PIN_ENCODER_A1 = 22;
const int PIN_ENCODER_B1 = 23;
const int PIN_ENCODER_A2 = 24;
const int PIN_ENCODER_B2 = 25;
const int PIN_ENCODER_A3 = 26;
const int PIN_ENCODER_B3 = 27;
const int PIN_ENCODER_A4 = 28;
const int PIN_ENCODER_B4 = 29;
const int PIN_ENCODER_A5 = 30;
const int PIN_ENCODER_B5 = 31;
const int PIN_ENCODER_A6 = 32;
const int PIN_ENCODER_B6 = 33;
const int PIN_ENCODER_A7 = 34;
const int PIN_ENCODER_B7 = 35;



const int PIN_MOTO1_DIR = 2;
const int PIN_MOTO1_PUL = 3;
const int PIN_MOTO2_DIR = 4;
const int PIN_MOTO2_PUL = 5;
const int PIN_MOTO3_DIR = 6;
const int PIN_MOTO3_PUL = 7;
const int PIN_MOTO4_DIR = 8;
const int PIN_MOTO4_PUL = 9;
const int PIN_MOTO5_DIR = 10;
const int PIN_MOTO5_PUL = 11;
const int PIN_MOTO6_DIR = 12;
const int PIN_MOTO6_PUL = 13;
const int PIN_MOTO7_DIR = 14;
const int PIN_MOTO7_PUL = 15;


const int PIN_LIMITER1_A = 54;
const int PIN_LIMITER1_B = 55;
const int PIN_LIMITER2_A = 56;
const int PIN_LIMITER2_B = 57;
const int PIN_LIMITER3_A = 58;
const int PIN_LIMITER3_B = 59;
const int PIN_LIMITER4_A = 60;
const int PIN_LIMITER4_B = 61;
const int PIN_LIMITER5_A = 62;
const int PIN_LIMITER5_B = 63;
const int PIN_LIMITER6_A = 64;
const int PIN_LIMITER6_B = 65;
const int PIN_LIMITER7_A = 52;
const int PIN_LIMITER7_B = 53;

const int PIN_EMERGENCY_STOP = 50;

// Arduino ADK interrupt
//const int PIN_ENCODER_A = 2;
//const int PIN_ENCODER_B = 3;


Encoder amt_encoder1(PIN_ENCODER_A1, PIN_ENCODER_B1);
Encoder amt_encoder2(PIN_ENCODER_A2, PIN_ENCODER_B2);
Encoder amt_encoder3(PIN_ENCODER_A3, PIN_ENCODER_B3);
Encoder amt_encoder4(PIN_ENCODER_A4, PIN_ENCODER_B4);
Encoder amt_encoder5(PIN_ENCODER_A5, PIN_ENCODER_B5);
Encoder amt_encoder6(PIN_ENCODER_A6, PIN_ENCODER_B6);
Encoder amt_encoder7(PIN_ENCODER_A7, PIN_ENCODER_B7);


const int MAIN_PULSE_DELAY = 300;


/*----------------------------------------------------------------------------*/

enum JointID
{
  JOINT_S1,
  JOINT_S2,
  JOINT_E1,
  JOINT_E2,
  JOINT_W1,
  JOINT_W2,
  JOINT_G
};


struct Command
{
  Command()
  {
    // nothing
  }

  Command(int _parallel_stage, int _motion_steps)
    : parallel_stage(_parallel_stage)
    , motion_steps(_motion_steps)
  {
    // nothing
  }

  int parallel_stage;
  int motion_steps;
};


const int JOINT_COMMAND_QUEUE_SIZE = 64;

struct CommandQueue
{
  CommandQueue()
    : first(0)
    , last(0)
  {
    // nothing
  }

  bool empty(void)
  {
    return (first == last);
  }

  void queue_Command(Command command)
  {
    Commands[last] = command;
    ++last;
    last %= JOINT_COMMAND_QUEUE_SIZE;
  };

  void dequeue_Command(void)
  {
    ++first;
    first %= JOINT_COMMAND_QUEUE_SIZE;
  }

  int first;
  int last;

  Command Commands[JOINT_COMMAND_QUEUE_SIZE];
};

CommandQueue command_Queue_S1;
CommandQueue command_Queue_S2;
CommandQueue command_Queue_E1;
CommandQueue command_Queue_E2;
CommandQueue command_Queue_W1;
CommandQueue command_Queue_W2;
CommandQueue command_Queue_G;


/*----------------------------------------------------------------------------*/

const double SPEED_UNDEFINED = -1.0;

const double JOINT_S1_TIMER_STOP_PERIOD = 65536.0;
const double JOINT_S1_TIMER_SLOW_PERIOD = 32.0;
const double JOINT_S1_TIMER_FAST_PERIOD = 6.0;
const double JOINT_S1_SLOW_SPEED = (1 / JOINT_S1_TIMER_SLOW_PERIOD);
const double JOINT_S1_FAST_SPEED = (1 / JOINT_S1_TIMER_FAST_PERIOD);
const int JOINT_S1_STEPS_TO_FAST_SPEED = 128;

/*
const double JOINT_S1_TIMER_SLOW_PERIOD = 32.0;
const double JOINT_S1_TIMER_FAST_PERIOD = 8.0;
const int JOINT_S1_FULL_SPEED_STEPS = 64;
*/

const int JOINT_S1_LIMITER_HIT_TRESHOLD = 4;
const int JOINT_S1_LIMITER_HIT_PERIOD = 16;
const int JOINT_S1_AWAY_LIMITER_STEPS = 32;

/* rev.1
  const double JOINT_S2_TIMER_SLOW_PERIOD = 32.0;
  const double JOINT_S2_TIMER_FAST_PERIOD = 16.0;
  const int JOINT_S2_FULL_SPEED_STEPS = 64;
*/

const double JOINT_S2_TIMER_STOP_PERIOD = 65536.0;
const double JOINT_S2_TIMER_SLOW_PERIOD = 64.0;
const double JOINT_S2_TIMER_FAST_PERIOD = 4.0;
const double JOINT_S2_SLOW_SPEED = (1 / JOINT_S2_TIMER_SLOW_PERIOD);
const double JOINT_S2_FAST_SPEED = (1 / JOINT_S2_TIMER_FAST_PERIOD);
const int JOINT_S2_STEPS_TO_FAST_SPEED = 128;


const int JOINT_S2_LIMITER_HIT_TRESHOLD = 4;
const int JOINT_S2_LIMITER_HIT_PERIOD = 16;
const int JOINT_S2_AWAY_LIMITER_STEPS = 32;

const double JOINT_E1_TIMER_STOP_PERIOD = 65536.0;
const double JOINT_E1_TIMER_SLOW_PERIOD = 16.0;
const double JOINT_E1_TIMER_FAST_PERIOD = 2.0;
const double JOINT_E1_SLOW_SPEED = (1 / JOINT_E1_TIMER_SLOW_PERIOD);
const double JOINT_E1_FAST_SPEED = (1 / JOINT_E1_TIMER_FAST_PERIOD);
const int JOINT_E1_STEPS_TO_FAST_SPEED = 128;
/*
const double JOINT_E1_TIMER_SLOW_PERIOD = 16.0;
const double JOINT_E1_TIMER_FAST_PERIOD = 2.0;
const int JOINT_E1_FULL_SPEED_STEPS = 128;
*/

const int JOINT_E1_LIMITER_HIT_TRESHOLD = 4;
const int JOINT_E1_LIMITER_HIT_PERIOD = 16;
const int JOINT_E1_AWAY_LIMITER_STEPS = 32;


const double JOINT_E2_TIMER_STOP_PERIOD = 65536.0;
const double JOINT_E2_TIMER_SLOW_PERIOD = 16.0;
const double JOINT_E2_TIMER_FAST_PERIOD = 3.0;
const double JOINT_E2_SLOW_SPEED = (1 / JOINT_E2_TIMER_SLOW_PERIOD);
const double JOINT_E2_FAST_SPEED = (1 / JOINT_E2_TIMER_FAST_PERIOD);
const int JOINT_E2_STEPS_TO_FAST_SPEED = 128;
/*
const double JOINT_E2_TIMER_SLOW_PERIOD = 16.0;
const double JOINT_E2_TIMER_FAST_PERIOD = 3.0;
const int JOINT_E2_FULL_SPEED_STEPS = 128;
*/

const int JOINT_E2_LIMITER_HIT_TRESHOLD = 4;
const int JOINT_E2_LIMITER_HIT_PERIOD = 16;
const int JOINT_E2_AWAY_LIMITER_STEPS = 32;


const double JOINT_W1_TIMER_STOP_PERIOD = 65536.0;
const double JOINT_W1_TIMER_SLOW_PERIOD = 16.0;
const double JOINT_W1_TIMER_FAST_PERIOD = 2.0;
const double JOINT_W1_SLOW_SPEED = (1 / JOINT_W1_TIMER_SLOW_PERIOD);
const double JOINT_W1_FAST_SPEED = (1 / JOINT_W1_TIMER_FAST_PERIOD);
const int JOINT_W1_STEPS_TO_FAST_SPEED = 128;
/*
const double JOINT_W1_TIMER_SLOW_PERIOD = 16.0;
const double JOINT_W1_TIMER_FAST_PERIOD = 2.0;
const int JOINT_W1_FULL_SPEED_STEPS = 128;
*/
const int JOINT_W1_LIMITER_HIT_TRESHOLD = 4;
const int JOINT_W1_LIMITER_HIT_PERIOD = 16;
const int JOINT_W1_AWAY_LIMITER_STEPS = 32;


const double JOINT_W2_TIMER_STOP_PERIOD = 65536.0;
const double JOINT_W2_TIMER_SLOW_PERIOD = 32.0;
const double JOINT_W2_TIMER_FAST_PERIOD = 4.0;
const double JOINT_W2_SLOW_SPEED = (1 / JOINT_W2_TIMER_SLOW_PERIOD);
const double JOINT_W2_FAST_SPEED = (1 / JOINT_W2_TIMER_FAST_PERIOD);
const int JOINT_W2_STEPS_TO_FAST_SPEED = 64;
/*
const double JOINT_W2_TIMER_SLOW_PERIOD = 16.0;
const double JOINT_W2_TIMER_FAST_PERIOD = 2.0;
const int JOINT_W2_FULL_SPEED_STEPS = 128;
*/
const int JOINT_W2_LIMITER_HIT_TRESHOLD = 4;
const int JOINT_W2_LIMITER_HIT_PERIOD = 16;
const int JOINT_W2_AWAY_LIMITER_STEPS = 32;


const double JOINT_G_TIMER_STOP_PERIOD = 65536.0;
const double JOINT_G_TIMER_SLOW_PERIOD = 8.0;
const double JOINT_G_TIMER_FAST_PERIOD = 2.0;
const double JOINT_G_SLOW_SPEED = (1 / JOINT_G_TIMER_SLOW_PERIOD);
const double JOINT_G_FAST_SPEED = (1 / JOINT_G_TIMER_FAST_PERIOD);
const int JOINT_G_STEPS_TO_FAST_SPEED = 128;
/*
const double JOINT_G_TIMER_SLOW_PERIOD = 8.0;
const double JOINT_G_TIMER_FAST_PERIOD = 2.0;
const int JOINT_G_FULL_SPEED_STEPS = 128;
*/

const int JOINT_G_LIMITER_HIT_TRESHOLD = 4;
const int JOINT_G_LIMITER_HIT_PERIOD = 16;
const int JOINT_G_AWAY_LIMITER_STEPS = 32;


double calc_Period2Speed(double timer_period)
{
  return (1 / timer_period);
}


double calc_Speed2Period(double joint_speed)
{
  return (1 / joint_speed);
}


int calc_Steps2Time(int steps, double timer_period)
{
  return timer_period * steps;
}


int calc_Time2Steps(int time, double timer_period)
{
  return time / timer_period;
}


int calc_TimeToSpeed(double final_speed, int steps_to_speed)
{
  return steps_to_speed / (0.5 * final_speed);
}


int calc_FlatMotionTime(double final_speed, int flat_steps)
{
  return flat_steps * calc_Speed2Period(final_speed);
}


int calc_SmoothMotionTimes(double final_speed, int total_steps, int steps_to_speed, int &time_to_speed)
{
  time_to_speed = calc_TimeToSpeed(final_speed, steps_to_speed);
  int time_on_flat_speed = calc_FlatMotionTime(final_speed, total_steps - 2 * steps_to_speed);

  return 2 * time_to_speed + time_on_flat_speed;
}


double calc_CurrentSpeed(int time, int time_to_speed, double final_speed)
{
  return (0.5 * (sin(((time / (double)time_to_speed) - 0.5) * PI) + 1) *  final_speed);
}


int calc_SmoothStepsToSpeed(int total_steps, double top_speed, int steps_to_top_speed, double &final_speed)
{
  if (total_steps > 2 * steps_to_top_speed)
  {
    final_speed = top_speed;
    return steps_to_top_speed;
  }
  else
  {
    final_speed = top_speed * ((double)total_steps / (2.0 * steps_to_top_speed));
    return (total_steps / 2);
  }
}


double calc_SpeedFactorForTime(int total_steps, double final_speed, int steps_to_speed, int total_time)
{  
  if (abs(total_steps) > 2 * steps_to_speed)
  {
    return ((abs(total_steps) + 2.0 * steps_to_speed) / (final_speed * total_time));
  }
  else
  {
    //total_time = (4.0 * steps_to_speed) /  (f * final_speed);
    return (4.0 * steps_to_speed / (final_speed * total_time));
  }
}


enum JointState
{
  JOINT_STATE_STEPPER_HOLDING,
  JOINT_STATE_STEPPER_START_MOVING,
  JOINT_STATE_STEPPER_MOVING_FORWARD,
  JOINT_STATE_STEPPER_HITTING_LIMITER,
  JOINT_STATE_STEPPER_MOVING_AWAY_LIMITER,

  JOINT_STATE_COMPENSATION,
  JOINT_STATE_FEEDBACK
};


struct Joint
{
  Joint()
    : state(JOINT_STATE_STEPPER_HOLDING)
    , limiter_hit_period(0)
    , limiter_hit_count(0)
    , time_to_speed(0)
    , steps_to_speed(0)
    , total_motion_time(0)
    , final_speed(SPEED_UNDEFINED)
    , speed_factor(1.0)
    , motion_steps_remaining(0)
    , total_steps_made(0)
    , executed_motion_steps(0)
  {
    // nothing
  }

  double calc_FinalSpeed(void)
  {
    return (speed_factor * final_speed);
  }

  void equalize_MotionTime(int common_total_motion_time)
  {
    if (total_motion_time > 0)
    {
      speed_factor = calc_SpeedFactorForTime(abs(motion_steps_remaining), final_speed, steps_to_speed, common_total_motion_time);
      total_motion_time = common_total_motion_time;
      time_to_speed /= speed_factor;
    }
  }

  void reset_MotionTimeEqualization(void)
  {
    speed_factor = 1.0;
  }

  JointState state;

  int discrete_time;
  int total_motion_time;
  double assumed_time;
  double current_period;

  int time_to_speed;
  int steps_to_speed;
  double final_speed;
  double speed_factor;

  int motion_steps_remaining;
  int total_steps_made;
  int executed_motion_steps;

  int limiter_hit_period;
  int limiter_hit_count;
};

const int PARALLEL_STAGE_PROMOTION_STEPS = 4;

int active_joints_count = 0;

int active_parallel_stage = 1;
int parallel_stage_promotion = 0;

Joint joint_S1;
Joint joint_S2;
Joint joint_E1;
Joint joint_E2;
Joint joint_W1;
Joint joint_W2;
Joint joint_G;

const int SERIAL_COMMUNICATION_PERIOD = 16;
int serial_communication_timeout = 0;

enum MainState
{
  MAIN_STATE_UNDEFINED = 0,
  MAIN_STATE_STEPPER_IDLE = 1,
  MAIN_STATE_STEPPER_ACTIVE = 2,
  MAIN_STATE_STARTING_BATCH = 3
};


unsigned int main_state = MAIN_STATE_STEPPER_IDLE;

char input_buffer[512];
int input_rd = 0;


void setup()
{
  pinMode(PIN_LIMITER1_A, INPUT_PULLUP);
  pinMode(PIN_LIMITER1_B, INPUT_PULLUP);
  pinMode(PIN_LIMITER2_A, INPUT_PULLUP);
  pinMode(PIN_LIMITER2_B, INPUT_PULLUP);
  pinMode(PIN_LIMITER3_A, INPUT_PULLUP);
  pinMode(PIN_LIMITER3_B, INPUT_PULLUP);
  pinMode(PIN_LIMITER4_A, INPUT_PULLUP);
  pinMode(PIN_LIMITER4_B, INPUT_PULLUP);
  pinMode(PIN_LIMITER5_A, INPUT_PULLUP);
  pinMode(PIN_LIMITER5_B, INPUT_PULLUP);
  pinMode(PIN_LIMITER6_A, INPUT_PULLUP);
  pinMode(PIN_LIMITER6_B, INPUT_PULLUP);
  pinMode(PIN_LIMITER7_A, INPUT_PULLUP);
  pinMode(PIN_LIMITER7_B, INPUT_PULLUP);
  pinMode(PIN_EMERGENCY_STOP, INPUT_PULLUP);

  digitalWrite(PIN_LIMITER1_A, HIGH);
  digitalWrite(PIN_LIMITER1_B, HIGH);
  digitalWrite(PIN_LIMITER2_A, HIGH);
  digitalWrite(PIN_LIMITER2_B, HIGH);
  digitalWrite(PIN_LIMITER3_A, HIGH);
  digitalWrite(PIN_LIMITER3_B, HIGH);
  digitalWrite(PIN_LIMITER4_A, HIGH);
  digitalWrite(PIN_LIMITER4_B, HIGH);
  digitalWrite(PIN_LIMITER5_A, HIGH);
  digitalWrite(PIN_LIMITER5_B, HIGH);
  digitalWrite(PIN_LIMITER6_A, HIGH);
  digitalWrite(PIN_LIMITER6_B, HIGH);
  digitalWrite(PIN_LIMITER7_A, HIGH);
  digitalWrite(PIN_LIMITER7_B, HIGH);
  digitalWrite(PIN_EMERGENCY_STOP, HIGH);

  pinMode(PIN_MOTO1_PUL, OUTPUT);
  pinMode(PIN_MOTO1_DIR, OUTPUT);
  pinMode(PIN_MOTO2_PUL, OUTPUT);
  pinMode(PIN_MOTO2_DIR, OUTPUT);
  pinMode(PIN_MOTO3_PUL, OUTPUT);
  pinMode(PIN_MOTO3_DIR, OUTPUT);
  pinMode(PIN_MOTO4_PUL, OUTPUT);
  pinMode(PIN_MOTO4_DIR, OUTPUT);
  pinMode(PIN_MOTO5_PUL, OUTPUT);
  pinMode(PIN_MOTO5_DIR, OUTPUT);
  pinMode(PIN_MOTO6_PUL, OUTPUT);
  pinMode(PIN_MOTO6_DIR, OUTPUT);
  pinMode(PIN_MOTO7_PUL, OUTPUT);
  pinMode(PIN_MOTO7_DIR, OUTPUT);

  Serial.begin(115200);

  amt_encoder1.write(0);
  amt_encoder2.write(0);
  amt_encoder3.write(0);
  amt_encoder4.write(0);
  amt_encoder5.write(0);
  amt_encoder6.write(0);
  amt_encoder7.write(0);

  /*
    command_Queue_W1.queue_Command(Command(1, -2048));
    command_Queue_W1.queue_Command(Command(2, 2048));
    command_Queue_W1.queue_Command(Command(3, -2048));
    command_Queue_W1.queue_Command(Command(4, 2048));
  */
  /*
    command_Queue_G.queue_Command(Command(2, -1024));
    command_Queue_G.queue_Command(Command(3, 1024));
    command_Queue_G.queue_Command(Command(4, -1024));
  */
  /*
    command_Queue_E1.queue_Command(Command(1, -1024));
    command_Queue_E1.queue_Command(Command(2, 1024));
    command_Queue_E1.queue_Command(Command(3, -2048));
    command_Queue_E1.queue_Command(Command(8, 2048));

    command_Queue_E2.queue_Command(Command(4, -1024));
    command_Queue_E2.queue_Command(Command(5, 1024));
    command_Queue_E2.queue_Command(Command(6, -2048));
    command_Queue_E2.queue_Command(Command(7, 2048));

    command_Queue_W2.queue_Command(Command(1, -1024));
    command_Queue_W2.queue_Command(Command(2, 1024));
    command_Queue_W2.queue_Command(Command(3, -2048));
    command_Queue_W2.queue_Command(Command(4, 2048));
    command_Queue_W2.queue_Command(Command(5, -2048));
    command_Queue_W2.queue_Command(Command(6, 2048));
  */


  // First complete run (must start from stand)
  /*
    command_Queue_S2.queue_Command(Command(1, -512));
    command_Queue_E1.queue_Command(Command(1, -1024));

    command_Queue_S1.queue_Command(Command(2, -512));
    command_Queue_W2.queue_Command(Command(2, -1024));
    command_Queue_W1.queue_Command(Command(2, -2048));
    command_Queue_G.queue_Command(Command(2, 1024));
    command_Queue_E2.queue_Command(Command(2, -2048));

    command_Queue_E1.queue_Command(Command(3, -1024));
    command_Queue_E1.queue_Command(Command(4, 1024));

    command_Queue_E2.queue_Command(Command(5, 2048));
    command_Queue_G.queue_Command(Command(5, -1024));
    command_Queue_W1.queue_Command(Command(5, 2048));
    command_Queue_W2.queue_Command(Command(5, 1024));
    command_Queue_S1.queue_Command(Command(5, 512));

    command_Queue_E1.queue_Command(Command(6, 1024));
    command_Queue_S2.queue_Command(Command(6, 512));

    command_Queue_W2.queue_Command(Command(7, 2048));
    command_Queue_E1.queue_Command(Command(7, 1024));
    command_Queue_G.queue_Command(Command(7, -1024));
    command_Queue_W1.queue_Command(Command(7, 2048));

    command_Queue_W1.queue_Command(Command(8, -2048));
    command_Queue_G.queue_Command(Command(8, 1024));
    command_Queue_E1.queue_Command(Command(8, -1024));
    command_Queue_W2.queue_Command(Command(8, -2048));
  */


  /*
     Program Coca-cola-barman
  */
  /*----------------*/
  /*
    command_Queue_E1.queue_Command(Command(1, -1024));
    command_Queue_S2.queue_Command(Command(1, -256));

    command_Queue_G.queue_Command(Command(2, -2048));
    command_Queue_W2.queue_Command(Command(3, -3584));
    command_Queue_W1.queue_Command(Command(3, 4096));
    command_Queue_S1.queue_Command(Command(3, 700));

    command_Queue_W1.queue_Command(Command(4, 4096));
    command_Queue_E1.queue_Command(Command(4, 2048));

    command_Queue_E1.queue_Command(Command(5, -128));
    command_Queue_W1.queue_Command(Command(5, 640));
    command_Queue_S2.queue_Command(Command(5, 160));

    command_Queue_G.queue_Command(Command(6, 2048));
    command_Queue_S2.queue_Command(Command(7, -256));
    command_Queue_E1.queue_Command(Command(7, -256));
    command_Queue_W1.queue_Command(Command(7, -1024));
    command_Queue_S1.queue_Command(Command(7, -320));

    command_Queue_E2.queue_Command(Command(8, 1024 + 2048 + 1024));
    command_Queue_W2.queue_Command(Command(8, 2048 + 1024 + 512));
    command_Queue_W2.queue_Command(Command(9, -2048 - 1024 - 512));
    command_Queue_E2.queue_Command(Command(9, -1024 - 2048 - 1024));

    command_Queue_E2.queue_Command(Command(10, -2048));
    command_Queue_E2.queue_Command(Command(11, 2048));

    command_Queue_S1.queue_Command(Command(12, 320));
    command_Queue_W1.queue_Command(Command(12, 1024));
    command_Queue_E1.queue_Command(Command(13, 256));
    command_Queue_S2.queue_Command(Command(13, 256));
    command_Queue_G.queue_Command(Command(14, -2048));

    command_Queue_S2.queue_Command(Command(15, -160));
    command_Queue_W1.queue_Command(Command(15, -640));
    command_Queue_E1.queue_Command(Command(15, 128));

    command_Queue_E1.queue_Command(Command(16, -2048));
    command_Queue_W1.queue_Command(Command(16, -4096));

    command_Queue_S1.queue_Command(Command(17, -700));
    command_Queue_W1.queue_Command(Command(17, -4096));
    command_Queue_W2.queue_Command(Command(17, 3584));
    command_Queue_G.queue_Command(Command(18, 2048));

    command_Queue_E1.queue_Command(Command(19, 1024));
    command_Queue_S2.queue_Command(Command(19, 256));
  */
  /*----------------*/


  /*
     Program Japano-vzperac
  */
  /*----------------*/
  /*
    command_Queue_E1.queue_Command(Command(1, -1024));
    command_Queue_S2.queue_Command(Command(1, -256));
    command_Queue_G.queue_Command(Command(2, -2048));

    command_Queue_S2.queue_Command(Command(3, 320));
    command_Queue_E1.queue_Command(Command(3, 640));
    command_Queue_W1.queue_Command(Command(3, 1024));
    command_Queue_G.queue_Command(Command(4, 2048));

    command_Queue_E1.queue_Command(Command(5, -2048));
    command_Queue_W1.queue_Command(Command(5, 4096));
    command_Queue_S2.queue_Command(Command(5, -256));

    command_Queue_S2.queue_Command(Command(6, 256));
    command_Queue_W1.queue_Command(Command(6, -4096));
    command_Queue_E1.queue_Command(Command(6, 2048));

    command_Queue_G.queue_Command(Command(7, -2048));
    command_Queue_W1.queue_Command(Command(8, -1024));
    command_Queue_E1.queue_Command(Command(8, -640));
    command_Queue_S2.queue_Command(Command(8, -320));

    command_Queue_G.queue_Command(Command(9, 2048));
    command_Queue_S2.queue_Command(Command(10, 256));
    command_Queue_E1.queue_Command(Command(10, 1024));
  */
  /*----------------*/

  /*
     Program Horizontal flex
  */
  /*----------------*/
  /*
    command_Queue_E1.queue_Command(Command(1, -1024 - 256));
    command_Queue_S2.queue_Command(Command(1, -128));

    command_Queue_W1.queue_Command(Command(2, 2048 + 1024));
    command_Queue_E2.queue_Command(Command(3, -1024 - 2048 - 2048));
    command_Queue_W2.queue_Command(Command(3, -2048 - 1024 - 1024));
    command_Queue_W2.queue_Command(Command(4, 2048 + 1024 + 1024));
    command_Queue_E2.queue_Command(Command(4, 1024 + 2048 + 2048));
    command_Queue_W1.queue_Command(Command(5, -2048 - 1024));

    command_Queue_S2.queue_Command(Command(6, 128));
    command_Queue_E1.queue_Command(Command(6, 1024 + 256));
  */

  /*
     Program Vertical flex
  */
  /*----------------*/
  /*
    command_Queue_E1.queue_Command(Command(1, -1024 - 1024 - 1024 - 256));
    command_Queue_S2.queue_Command(Command(1, -64));

    command_Queue_W1.queue_Command(Command(2, 2048 + 1024));
    command_Queue_E2.queue_Command(Command(3, -1024 - 2048 - 2048));
    command_Queue_W2.queue_Command(Command(3, -2048 - 1024 - 1024));
    command_Queue_G.queue_Command(Command(3, -2048));
    command_Queue_S1.queue_Command(Command(3, -800));
    command_Queue_S1.queue_Command(Command(4, 800));
    command_Queue_G.queue_Command(Command(4, 2048));
    command_Queue_W2.queue_Command(Command(4, 2048 + 1024 + 1024));
    command_Queue_E2.queue_Command(Command(4, 1024 + 2048 + 2048));
    command_Queue_W1.queue_Command(Command(5, -2048 - 1024));

    command_Queue_S2.queue_Command(Command(6, 64));
    command_Queue_E1.queue_Command(Command(6, 1024 + 1024 + 1024 + 256));
  */

  /*
     Program Blocks World 3
  */
  /*
    command_Queue_E1.queue_Command(Command(1, -1024));
    command_Queue_S2.queue_Command(Command(1, -256));

    command_Queue_W2.queue_Command(Command(2, 2048 - 512));
    command_Queue_S1.queue_Command(Command(2, 256 + 256 + 128));
    command_Queue_W1.queue_Command(Command(2, -1024));

    command_Queue_G.queue_Command(Command(3, -2048));

    command_Queue_E1.queue_Command(Command(4, 512 + 256 + 256 + 128 + 64));
    command_Queue_W1.queue_Command(Command(4, 512 + 256));

    command_Queue_G.queue_Command(Command(5, 2048));

    command_Queue_S2.queue_Command(Command(6, -128));
    //command_Queue_E1.queue_Command(Command(6, -1024));
    command_Queue_W1.queue_Command(Command(6, 1024));

    command_Queue_W1.queue_Command(Command(7, 2048 + 2048 + 2048 + 1024));
    command_Queue_E1.queue_Command(Command(8, 1024));
    command_Queue_W2.queue_Command(Command(8, 2048));

    command_Queue_S2.queue_Command(Command(9, 256));
    command_Queue_E1.queue_Command(Command(9, -1024 + 128));
    command_Queue_S1.queue_Command(Command(9, -256));

    command_Queue_E1.queue_Command(Command(10, 256));
    command_Queue_W1.queue_Command(Command(10, -64));
    command_Queue_S2.queue_Command(Command(10, 64));

    command_Queue_G.queue_Command(Command(11, -2048));

    command_Queue_S2.queue_Command(Command(12, -64));
    command_Queue_W1.queue_Command(Command(12, 64));
    command_Queue_E1.queue_Command(Command(12, -256));

    command_Queue_S1.queue_Command(Command(13, 256));
    command_Queue_E1.queue_Command(Command(13, 1024 - 128));
    command_Queue_S2.queue_Command(Command(13, -256));

    command_Queue_W2.queue_Command(Command(14, -2048));
    command_Queue_E1.queue_Command(Command(14, -1024));
    command_Queue_W1.queue_Command(Command(15, -2048 - 2048 - 2048 - 1024));

    command_Queue_W1.queue_Command(Command(16, -1024));
    //command_Queue_E1.queue_Command(Command(16, 1024));
    command_Queue_S2.queue_Command(Command(16, 128));
    command_Queue_G.queue_Command(Command(16, -2048));

    command_Queue_W1.queue_Command(Command(17, -512 - 256));
    command_Queue_E1.queue_Command(Command(17, -512 - 256 - 256 - 128 - 64));

    command_Queue_W1.queue_Command(Command(18, 1024));
    command_Queue_S1.queue_Command(Command(18, -256 - 256 - 128));
    command_Queue_W2.queue_Command(Command(18, -2048 + 512));

    command_Queue_G.queue_Command(Command(19, -2048));
    command_Queue_G.queue_Command(Command(19, 2048));

    command_Queue_S2.queue_Command(Command(20, 256));
    command_Queue_E1.queue_Command(Command(20, 1024));
  */

  /*
     Program Wrist flex
  */
  /*----------------*/
  /*
    command_Queue_W2.queue_Command(Command(1, 4096));
    command_Queue_W2.queue_Command(Command(2, -4096));
    command_Queue_W2.queue_Command(Command(3, 4096));
    command_Queue_W2.queue_Command(Command(4, -4096));
    command_Queue_W2.queue_Command(Command(5, 4096));
    command_Queue_W2.queue_Command(Command(6, -4096));
  */

  /*
     Program Elbow flex
  */
  /*----------------*/
  /*
    command_Queue_E2.queue_Command(Command(1, -4096));
    command_Queue_E2.queue_Command(Command(2, 4096));
    command_Queue_E2.queue_Command(Command(3, -4096));
    command_Queue_E2.queue_Command(Command(4, 4096));
  */

  /*
     Program Callibration, Inverse Kinematics
  */

  /*

    //command_Queue_E1.queue_Command(Command(1, -256));

    command_Queue_S1.queue_Command(Command(1, 103));
    command_Queue_S2.queue_Command(Command(1, 280));

    command_Queue_E1.queue_Command(Command(1, -286));
    command_Queue_E2.queue_Command(Command(1, -482));

    command_Queue_W1.queue_Command(Command(1, -1749));
    command_Queue_W2.queue_Command(Command(1, 1121));

    command_Queue_G.queue_Command(Command(2, -1024));
    command_Queue_G.queue_Command(Command(3, 1024));

    command_Queue_S1.queue_Command(Command(4, -103));
    command_Queue_S2.queue_Command(Command(4, -280));

    command_Queue_E1.queue_Command(Command(4, 286));
    command_Queue_E2.queue_Command(Command(4, 482));

    command_Queue_W1.queue_Command(Command(4, 1749));
    command_Queue_W2.queue_Command(Command(4, -1121));
  */
  /*
    command_Queue_S1.queue_Command(Command(1, -538));
    command_Queue_S2.queue_Command(Command(1, 148));

    command_Queue_E1.queue_Command(Command(1, 0));
    command_Queue_E2.queue_Command(Command(1, 2295));

    command_Queue_W1.queue_Command(Command(1, -1245));
    command_Queue_W2.queue_Command(Command(1, -968));

    command_Queue_G.queue_Command(Command(2, -1024));
    command_Queue_G.queue_Command(Command(3, 1024));

    command_Queue_S1.queue_Command(Command(4, 538));
    command_Queue_S2.queue_Command(Command(4, -148));

    command_Queue_E1.queue_Command(Command(4, 0));
    command_Queue_E2.queue_Command(Command(4, -2295));

    command_Queue_W1.queue_Command(Command(4, 1245));
    command_Queue_W2.queue_Command(Command(4, 968));
  */
  /*
    command_Queue_S1.queue_Command(Command(1, 89));
    command_Queue_S2.queue_Command(Command(1, 280));

    command_Queue_E1.queue_Command(Command(1, -344));
    command_Queue_E2.queue_Command(Command(1, -416));

    command_Queue_W1.queue_Command(Command(1, -1788));
    command_Queue_W2.queue_Command(Command(1, 72));

    command_Queue_G.queue_Command(Command(2, -1024));
    command_Queue_G.queue_Command(Command(3, 1024));

    command_Queue_S1.queue_Command(Command(4, -89));
    command_Queue_S2.queue_Command(Command(4, -280));

    command_Queue_E1.queue_Command(Command(4, 344));
    command_Queue_E2.queue_Command(Command(4, 416));

    command_Queue_W1.queue_Command(Command(4, 1788));
    command_Queue_W2.queue_Command(Command(4, -72));
  */
  /*
    command_Queue_S1.queue_Command(Command(1, 172));
    command_Queue_S2.queue_Command(Command(1, 280));

    command_Queue_E1.queue_Command(Command(1, 1400));
    command_Queue_E2.queue_Command(Command(1, 4311));

    command_Queue_W1.queue_Command(Command(1, -2927));
    command_Queue_W2.queue_Command(Command(1, 2631));

    command_Queue_G.queue_Command(Command(2, -1024));
    command_Queue_G.queue_Command(Command(3, 1024));

    command_Queue_S1.queue_Command(Command(4, -172));
    command_Queue_S2.queue_Command(Command(4, -280));

    command_Queue_E1.queue_Command(Command(4, -1400));
    command_Queue_E2.queue_Command(Command(4, -4311));

    command_Queue_W1.queue_Command(Command(4, 2927));
    command_Queue_W2.queue_Command(Command(4, -2631));
  */
  /*
    command_Queue_S1.queue_Command(Command(1, 10));
    command_Queue_S2.queue_Command(Command(1, 28));

    command_Queue_E1.queue_Command(Command(1, -28));
    command_Queue_E2.queue_Command(Command(1, 48));

    command_Queue_W1.queue_Command(Command(1, 174));
    command_Queue_W2.queue_Command(Command(1, 112));


    command_Queue_S1.queue_Command(Command(2, -10));
    command_Queue_S2.queue_Command(Command(2, -28));

    command_Queue_E1.queue_Command(Command(2, 28));
    command_Queue_E2.queue_Command(Command(2, -48));

    command_Queue_W1.queue_Command(Command(2, -174));
    command_Queue_W2.queue_Command(Command(2, -112));
  */
  /*
    command_Queue_G.queue_Command(Command(2, -2048));
    command_Queue_G.queue_Command(Command(2, 2048));

    command_Queue_S2.queue_Command(Command(3, 256));
    command_Queue_E1.queue_Command(Command(3, 2048));
  */

  /*----------------*/


  /*
     Program RR1 rev.2 first test - incomplete run, only J-S2 test
  */
  /*----------------*/
  /*
    command_Queue_S2.queue_Command(Command(1, 32));  // up (opposite w.r.t. rev.1)
    command_Queue_S2.queue_Command(Command(2, -32)); // down (opposite w.r.t. rev.1)
  */
  /*----------------*/

  /*
     Program RR1 rev.2 second test - incomplete run, only J-E1 test
  */
  /*----------------*/
  /*
    command_Queue_S2.queue_Command(Command(1, 16));  // down
    command_Queue_S2.queue_Command(Command(2, -16)); // up
  */
  /*----------------*/


  /*
     Program RR1 rev.2 second test - folding
  */
  /*----------------*/
  /*
    //command_Queue_S2.queue_Command(Command(1, 64));  // up
    //command_Queue_S2.queue_Command(Command(2, -16)); // down
  */
  /*----------------*/


  /*
     Program RR1 rev.2 third test - incomplete run, only Gripper test
  */
  /*----------------*/
  /*
    //command_Queue_G.queue_Command(Command(1, -16));  // close
    //command_Queue_G.queue_Command(Command(2, -3)); // open
  */
  /*----------------*/


  /*
     Program RR1 rev.2 fourth test - encoder test
  */
  /*----------------*/
  /*
    command_Queue_G.queue_Command(Command(1, -8));  // close
    command_Queue_G.queue_Command(Command(2, 8)); // open
  */
  /*
    command_Queue_E1.queue_Command(Command(1, -2));  // left
    command_Queue_E1.queue_Command(Command(2, 2)); // right
  */

  /*----------------*/

  /*
     Program RR1 rev.2 fast unfolding test
  */
  /*----------------*/
  /*
    command_Queue_E1.queue_Command(Command(1, -4096 - 2048));  // up
    command_Queue_S2.queue_Command(Command(1, -3072 - 1024));  // up

    command_Queue_E2.queue_Command(Command(2, 1024));
    command_Queue_G.queue_Command(Command(2, -512 - 256));
    command_Queue_G.queue_Command(Command(3, 512 + 256));

    command_Queue_E2.queue_Command(Command(3, -1024));

    command_Queue_S2.queue_Command(Command(3, 3072 + 1024)); // down
    command_Queue_E1.queue_Command(Command(3, 1024 + 2048)); // down

    command_Queue_E1.queue_Command(Command(4, 3072)); // down
  */
  /*
    command_Queue_E1.queue_Command(Command(1, -4096));
    command_Queue_S2.queue_Command(Command(1, -3072));

    command_Queue_S2.queue_Command(Command(2, 3072));
    command_Queue_E1.queue_Command(Command(2, 4096));
  */

  /*
     Program RR1 rev.2 limiter tests
  */
  /*----------------*/
  /*
    command_Queue_G.queue_Command(Command(1, -2048));
    command_Queue_G.queue_Command(Command(2, 2048));
    command_Queue_G.queue_Command(Command(3, -2048));
    command_Queue_G.queue_Command(Command(4, 2048));
  */
  /*
    command_Queue_W2.queue_Command(Command(1, -2048));
    command_Queue_W2.queue_Command(Command(2, 2048));
    command_Queue_W2.queue_Command(Command(3, -2048));
    command_Queue_W2.queue_Command(Command(4, 2048));
  */
  /*
    command_Queue_E2.queue_Command(Command(1, -4096));
    command_Queue_E2.queue_Command(Command(2, 4096));
    command_Queue_E2.queue_Command(Command(3, -4096));
    command_Queue_E2.queue_Command(Command(4, 4096));
  */
  /*
    command_Queue_W1.queue_Command(Command(1, -8192));
    command_Queue_W1.queue_Command(Command(2, 8192));
    command_Queue_W1.queue_Command(Command(3, -8192));
    command_Queue_W1.queue_Command(Command(4, 8192));
  */
  /*
    command_Queue_E1.queue_Command(Command(1, -2048));
    command_Queue_E1.queue_Command(Command(2, 2048));
    command_Queue_E1.queue_Command(Command(3, -2048));
    command_Queue_E1.queue_Command(Command(4, 2048));
  */
  /*
    command_Queue_S2.queue_Command(Command(1, -4096));
    command_Queue_S2.queue_Command(Command(2, 4096));
    command_Queue_S2.queue_Command(Command(3, -4096));
    command_Queue_S2.queue_Command(Command(4, 4096));
  */
  /*
    command_Queue_S1.queue_Command(Command(1, -2048));
    command_Queue_S1.queue_Command(Command(2, 2048));
    command_Queue_S1.queue_Command(Command(3, -2048));
    command_Queue_S1.queue_Command(Command(4,2048));
  */

  /*
    command_Queue_S2.queue_Command(Command(1, -256));
    command_Queue_E1.queue_Command(Command(1, -512));

    command_Queue_S2.queue_Command(Command(2, 256));
    command_Queue_E1.queue_Command(Command(2, 512));

    command_Queue_S2.queue_Command(Command(3, -256));
    command_Queue_E1.queue_Command(Command(3, -512));

    command_Queue_S2.queue_Command(Command(4, 256));
    command_Queue_E1.queue_Command(Command(4, 512));

    command_Queue_S2.queue_Command(Command(5, -256));
    command_Queue_E1.queue_Command(Command(5, -512));

    command_Queue_S2.queue_Command(Command(6, 256));
    command_Queue_E1.queue_Command(Command(6, 512));

    command_Queue_S2.queue_Command(Command(7, -256));
    command_Queue_E1.queue_Command(Command(7, -512));

    command_Queue_S2.queue_Command(Command(8, 256));
    command_Queue_E1.queue_Command(Command(8, 512));
  */
  /*
    command_Queue_S1.queue_Command(Command(1, -256));
    command_Queue_E1.queue_Command(Command(1, -256));

    command_Queue_S1.queue_Command(Command(2, 256));
    command_Queue_E1.queue_Command(Command(2, 256));

    command_Queue_S1.queue_Command(Command(3, -256));
    command_Queue_E1.queue_Command(Command(3, -256));

    command_Queue_S1.queue_Command(Command(4, 256));
    command_Queue_E1.queue_Command(Command(4, 256));

    command_Queue_S1.queue_Command(Command(5, -256));
    command_Queue_E1.queue_Command(Command(5, -256));

    command_Queue_S1.queue_Command(Command(6, 256));
    command_Queue_E1.queue_Command(Command(6, 256));

    command_Queue_S1.queue_Command(Command(7, -256));
    command_Queue_E1.queue_Command(Command(7, -256));

    command_Queue_S1.queue_Command(Command(8, 256));
    command_Queue_E1.queue_Command(Command(8, 256));
  */
  /*
    command_Queue_S2.queue_Command(Command(1, 256));
    command_Queue_S2.queue_Command(Command(2, -256));
    command_Queue_S2.queue_Command(Command(3, 256));
    command_Queue_S2.queue_Command(Command(4, -256));
  */

  /*----------------*/



}


void loop()
{
  bool joint_S1_pulsed = false;
  bool joint_S2_pulsed = false;
  bool joint_E1_pulsed = false;
  bool joint_E2_pulsed = false;
  bool joint_W1_pulsed = false;
  bool joint_W2_pulsed = false;
  bool joint_G_pulsed = false;

  /*
     int joint_S1_limiter_A = digitalRead(PIN_LIMITER6_B);
     int joint_S1_limiter_B = digitalRead(PIN_LIMITER6_A);
  */
  // S1 has swapped limiters, rev.2 has double swapped
  int joint_S1_limiter_A = digitalRead(PIN_LIMITER6_A);
  int joint_S1_limiter_B = digitalRead(PIN_LIMITER6_B);

  /*
    int joint_S2_limiter_A = digitalRead(PIN_LIMITER3_A);
    int joint_S2_limiter_B = digitalRead(PIN_LIMITER3_B);
  */

  // rev.2 has swapped limiters on S2
  int joint_S2_limiter_A = digitalRead(PIN_LIMITER3_B);
  int joint_S2_limiter_B = digitalRead(PIN_LIMITER3_A);

  int joint_E1_limiter_A = digitalRead(PIN_LIMITER5_A);
  int joint_E1_limiter_B = digitalRead(PIN_LIMITER5_B);

  int joint_E2_limiter_A = digitalRead(PIN_LIMITER4_A);
  int joint_E2_limiter_B = digitalRead(PIN_LIMITER4_B);

  /*
     int joint_W1_limiter_A = digitalRead(PIN_LIMITER2_A);
     int joint_W1_limiter_B = digitalRead(PIN_LIMITER2_B);
  */

  // rev.2 has swapped limiters on W1
  int joint_W1_limiter_A = digitalRead(PIN_LIMITER2_B);
  int joint_W1_limiter_B = digitalRead(PIN_LIMITER2_A);

  int joint_W2_limiter_A = digitalRead(PIN_LIMITER7_A);
  int joint_W2_limiter_B = digitalRead(PIN_LIMITER7_B);

  /*
    int joint_G_limiter_A = digitalRead(PIN_LIMITER1_A);
    int joint_G_limiter_B = digitalRead(PIN_LIMITER1_B);
  */

  // rev.2 has swapped limiters on G
  int joint_G_limiter_A = digitalRead(PIN_LIMITER1_B);
  int joint_G_limiter_B = digitalRead(PIN_LIMITER1_A);

  /*
     Serial.print("stage:");
     Serial.print(active_parallel_stage);
     Serial.print("\n");

     Serial.print("S1:");
     Serial.print(joint_S1.total_steps_made);
     Serial.print(",");

     Serial.print("S2:");
     Serial.print(joint_S2.total_steps_made);
     Serial.print(",");

     Serial.print("W1:");
     Serial.print(joint_W1.total_steps_made);
     Serial.print(",");

     Serial.print("W2:");
     Serial.print(joint_W2.total_steps_made);
     Serial.print(",");
  */

  if (joint_S1_limiter_A == HIGH)
  {
    joints_limiters_state |= J_S1_LIMITER_A_MASK;
  }
  if (joint_S1_limiter_B == HIGH)
  {
    joints_limiters_state |= J_S1_LIMITER_B_MASK;
  }  
  if (joint_S2_limiter_A == HIGH)
  {
    joints_limiters_state |= J_S2_LIMITER_A_MASK;
  }
  if (joint_S2_limiter_B == HIGH)
  {
    joints_limiters_state |= J_S2_LIMITER_B_MASK;
  }    
  
  if (joint_E1_limiter_A == HIGH)
  {
    joints_limiters_state |= J_E1_LIMITER_A_MASK;
  }
  if (joint_E1_limiter_B == HIGH)
  {
    joints_limiters_state |= J_E1_LIMITER_B_MASK;
  }  
  if (joint_E2_limiter_A == HIGH)
  {
    joints_limiters_state |= J_E2_LIMITER_A_MASK;
  }
  if (joint_E2_limiter_B == HIGH)
  {
    joints_limiters_state |= J_E2_LIMITER_B_MASK;
  }    

  if (joint_W1_limiter_A == HIGH)
  {
    joints_limiters_state |= J_W1_LIMITER_A_MASK;
  }
  if (joint_W1_limiter_B == HIGH)
  {
    joints_limiters_state |= J_W1_LIMITER_B_MASK;
  }  
  if (joint_W2_limiter_A == HIGH)
  {
    joints_limiters_state |= J_W2_LIMITER_A_MASK;
  }
  if (joint_W2_limiter_B == HIGH)
  {
    joints_limiters_state |= J_W2_LIMITER_B_MASK;
  }    

  if (joint_G_limiter_A == HIGH)
  {
    joints_limiters_state |= J_G_LIMITER_A_MASK;
  }
  if (joint_G_limiter_B == HIGH)
  {
    joints_limiters_state |= J_G_LIMITER_B_MASK;
  }    
  
  switch (main_state)
  {
      case MAIN_STATE_STEPPER_ACTIVE:
      {
        if (active_joints_count == 0)
        {
          if (++parallel_stage_promotion >= PARALLEL_STAGE_PROMOTION_STEPS)
          {
            parallel_stage_promotion = 0;
            ++active_parallel_stage;
            /*
            if (active_parallel_stage == command_Queue_S1.Commands[command_Queue_S1.first].parallel_stage)
            {
              int steps_to_speed = calc_SmoothStepsToSpeed(abs(command_Queue_S1.Commands[command_Queue_S1.first].motion_steps), JOINT_S1_FAST_SPEED, JOINT_S1_STEPS_TO_FAST_SPEED, joint_S1.calc_FinalSpeed());
              int total_motion_time = calc_SmoothMotionTimes(joint_S1.calc_FinalSpeed(), abs(command_Queue_S1.Commands[command_Queue_S1.first].motion_steps), steps_to_speed, time_to_speed);              
            }
            */
            main_state = MAIN_STATE_STEPPER_IDLE;
          }
        }
        break;
      }
      case MAIN_STATE_STEPPER_IDLE:
      {
        // doing nothing
        break;
      }
      default:
      {
        break;
      }
  }


  switch (joint_S1.state)
  {
      case JOINT_STATE_STEPPER_HOLDING:
      {
        if (!command_Queue_S1.empty())
        {
          if (active_parallel_stage == command_Queue_S1.Commands[command_Queue_S1.first].parallel_stage)
          {
            joint_S1.motion_steps_remaining = command_Queue_S1.Commands[command_Queue_S1.first].motion_steps;
            joint_S1.current_period = JOINT_S1_TIMER_SLOW_PERIOD;
            joint_S1.state = JOINT_STATE_STEPPER_MOVING_FORWARD;

            joint_S1.discrete_time = 0;
            joint_S1.assumed_time = 0.0;
            joint_S1.total_steps_made = 0;

            command_Queue_S1.dequeue_Command();
            ++active_joints_count;

            main_state = MAIN_STATE_STEPPER_ACTIVE;
          }
        }
        break;
      }
      case JOINT_STATE_STEPPER_MOVING_FORWARD:
      {
        if (++joint_S1.discrete_time - joint_S1.assumed_time > joint_S1.current_period)
        {
          if (joint_S1.motion_steps_remaining > 0)
          {
            digitalWrite(PIN_MOTO6_DIR, HIGH);
            digitalWrite(PIN_MOTO6_PUL, HIGH);
            --joint_S1.motion_steps_remaining;
            --joint_S1.executed_motion_steps;            
            ++joint_S1.total_steps_made;

            if (joint_S1_limiter_A == HIGH)
            {
              joint_S1.state = JOINT_STATE_STEPPER_HITTING_LIMITER;
              joint_S1.limiter_hit_period = JOINT_S1_LIMITER_HIT_PERIOD;
              joint_S1.limiter_hit_count = 1;
            }
          }
          else if (joint_S1.motion_steps_remaining < 0)
          {
            digitalWrite(PIN_MOTO6_DIR, LOW);
            digitalWrite(PIN_MOTO6_PUL, HIGH);
            ++joint_S1.motion_steps_remaining;
            ++joint_S1.executed_motion_steps;            
            ++joint_S1.total_steps_made;

            if (joint_S1_limiter_B == HIGH)
            {
              joint_S1.state = JOINT_STATE_STEPPER_HITTING_LIMITER;
              joint_S1.limiter_hit_period = JOINT_S1_LIMITER_HIT_PERIOD;
              joint_S1.limiter_hit_count = 1;;              
            }
          }
          else
          {
            joint_S1.state = JOINT_STATE_STEPPER_HOLDING;
            --active_joints_count;
          }
          joint_S1_pulsed = true;
          joint_S1.assumed_time += joint_S1.current_period;
        }
        break;
      }
      case JOINT_STATE_STEPPER_HITTING_LIMITER:
      {
        if (++joint_S1.discrete_time - joint_S1.assumed_time > joint_S1.current_period)
        {
          if (--joint_S1.limiter_hit_period <= 0)
          {
            joint_S1.state = JOINT_STATE_STEPPER_MOVING_FORWARD;
          }

          if (joint_S1.motion_steps_remaining > 0)
          {
            digitalWrite(PIN_MOTO6_DIR, HIGH);
            digitalWrite(PIN_MOTO6_PUL, HIGH);
            --joint_S1.motion_steps_remaining;
            --joint_S1.executed_motion_steps;            
            ++joint_S1.total_steps_made;

            if (joint_S1_limiter_A == HIGH)
            {
              if (++joint_S1.limiter_hit_count >= JOINT_S1_LIMITER_HIT_TRESHOLD)
              {
                joint_S1.state = JOINT_STATE_STEPPER_MOVING_AWAY_LIMITER;
                joint_S1.motion_steps_remaining = -JOINT_S1_AWAY_LIMITER_STEPS;
                joint_S1.current_period = JOINT_S1_TIMER_SLOW_PERIOD;
              }
              joint_S1.limiter_hit_period = JOINT_S1_LIMITER_HIT_PERIOD;
            }
          }
          else if (joint_S1.motion_steps_remaining < 0)
          {
            digitalWrite(PIN_MOTO6_DIR, LOW);
            digitalWrite(PIN_MOTO6_PUL, HIGH);
            ++joint_S1.motion_steps_remaining;
            ++joint_S1.executed_motion_steps;            
            ++joint_S1.total_steps_made;

            if (joint_S1_limiter_B == HIGH)
            {
              if (++joint_S1.limiter_hit_count >= JOINT_S1_LIMITER_HIT_TRESHOLD)
              {
                joint_S1.state = JOINT_STATE_STEPPER_MOVING_AWAY_LIMITER;
                joint_S1.motion_steps_remaining = JOINT_S1_AWAY_LIMITER_STEPS;
                joint_S1.current_period = JOINT_S1_TIMER_SLOW_PERIOD;
              }
              joint_S1.limiter_hit_period = JOINT_S1_LIMITER_HIT_PERIOD;
            }
          }
          else
          {
            joint_S1.state = JOINT_STATE_STEPPER_HOLDING;
            --active_joints_count;
          }
          joint_S1_pulsed = true;
          joint_S1.assumed_time += joint_S1.current_period;
        }
        break;
      }
      case JOINT_STATE_STEPPER_MOVING_AWAY_LIMITER:
      {
        if (++joint_S1.discrete_time - joint_S1.assumed_time > joint_S1.current_period)
        {
          if (joint_S1.motion_steps_remaining > 0)
          {
            digitalWrite(PIN_MOTO6_DIR, HIGH);
            digitalWrite(PIN_MOTO6_PUL, HIGH);
            --joint_S1.motion_steps_remaining;
            --joint_S1.executed_motion_steps;            
          }
          else if (joint_S1.motion_steps_remaining < 0)
          {
            digitalWrite(PIN_MOTO6_DIR, LOW);
            digitalWrite(PIN_MOTO6_PUL, HIGH);
            ++joint_S1.motion_steps_remaining;
            ++joint_S1.executed_motion_steps;            
          }
          else
          {
            joint_S1.state = JOINT_STATE_STEPPER_HOLDING;
            --active_joints_count;
          }
          joint_S1_pulsed = true;
          joint_S1.assumed_time += joint_S1.current_period;
        }
        break;
      }
      default:
      {
        break;
      }
  }


  switch (joint_S2.state)
  {
      case JOINT_STATE_STEPPER_HOLDING:
      {
        if (!command_Queue_S2.empty())
        {
          if (active_parallel_stage == command_Queue_S2.Commands[command_Queue_S2.first].parallel_stage)
          {
            joint_S2.motion_steps_remaining = command_Queue_S2.Commands[command_Queue_S2.first].motion_steps;
            joint_S2.current_period = JOINT_S2_TIMER_SLOW_PERIOD;
            joint_S2.state = JOINT_STATE_STEPPER_MOVING_FORWARD;

            joint_S2.discrete_time = 0;
            joint_S2.assumed_time = 0.0;
            joint_S2.total_steps_made = 0;

            command_Queue_S2.dequeue_Command();
            ++active_joints_count;

            main_state = MAIN_STATE_STEPPER_ACTIVE;
          }
        }
        break;
      }
      case JOINT_STATE_STEPPER_MOVING_FORWARD:
      {
        if (++joint_S2.discrete_time - joint_S2.assumed_time > joint_S2.current_period)
        {
          if (joint_S2.motion_steps_remaining > 0)
          {
            digitalWrite(PIN_MOTO3_DIR, HIGH);
            digitalWrite(PIN_MOTO3_PUL, HIGH);
            --joint_S2.motion_steps_remaining;
            --joint_S2.executed_motion_steps;            
            ++joint_S2.total_steps_made;

            if (joint_S2_limiter_A == HIGH)
            {
              joint_S2.state = JOINT_STATE_STEPPER_HITTING_LIMITER;
              joint_S2.limiter_hit_period = JOINT_S2_LIMITER_HIT_PERIOD;
              joint_S2.limiter_hit_count = 1;
            }
          }
          else if (joint_S2.motion_steps_remaining < 0)
          {
            digitalWrite(PIN_MOTO3_DIR, LOW);
            digitalWrite(PIN_MOTO3_PUL, HIGH);
            ++joint_S2.motion_steps_remaining;
            ++joint_S2.executed_motion_steps;
            ++joint_S2.total_steps_made;

            if (joint_S2_limiter_B == HIGH)
            {
              joint_S2.state = JOINT_STATE_STEPPER_HITTING_LIMITER;
              joint_S2.limiter_hit_period = JOINT_S2_LIMITER_HIT_PERIOD;
              joint_S2.limiter_hit_count = 1;
            }
          }
          else
          {
            joint_S2.state = JOINT_STATE_STEPPER_HOLDING;
            --active_joints_count;
          }
          joint_S2_pulsed = true;
          joint_S2.assumed_time += joint_S2.current_period;
        }
        break;
      }
      case JOINT_STATE_STEPPER_HITTING_LIMITER:
      {
        if (++joint_S2.discrete_time - joint_S2.assumed_time > joint_S2.current_period)
        {
          if (--joint_S2.limiter_hit_period <= 0)
          {
            joint_S2.state = JOINT_STATE_STEPPER_MOVING_FORWARD;
          }

          if (joint_S2.motion_steps_remaining > 0)
          {
            digitalWrite(PIN_MOTO3_DIR, HIGH);
            digitalWrite(PIN_MOTO3_PUL, HIGH);
            --joint_S2.motion_steps_remaining;
            --joint_S2.executed_motion_steps;
            ++joint_S2.total_steps_made;

            if (joint_S2_limiter_A == HIGH)
            {
              if (++joint_S2.limiter_hit_count >= JOINT_S2_LIMITER_HIT_TRESHOLD)
              {
                joint_S2.state = JOINT_STATE_STEPPER_MOVING_AWAY_LIMITER;
                joint_S2.motion_steps_remaining = -JOINT_S2_AWAY_LIMITER_STEPS;
                joint_S2.current_period = JOINT_S2_TIMER_SLOW_PERIOD;
              }
              joint_S2.limiter_hit_period = JOINT_S2_LIMITER_HIT_PERIOD;
            }
          }
          else if (joint_S2.motion_steps_remaining < 0)
          {
            digitalWrite(PIN_MOTO3_DIR, LOW);
            digitalWrite(PIN_MOTO3_PUL, HIGH);
            ++joint_S2.motion_steps_remaining;
            ++joint_S2.executed_motion_steps;
            ++joint_S2.total_steps_made;

            if (joint_S2_limiter_B == HIGH)
            {
              if (++joint_S2.limiter_hit_count >= JOINT_S2_LIMITER_HIT_TRESHOLD)
              {
                joint_S2.state = JOINT_STATE_STEPPER_MOVING_AWAY_LIMITER;
                joint_S2.motion_steps_remaining = JOINT_S2_AWAY_LIMITER_STEPS;
                joint_S2.current_period = JOINT_S2_TIMER_SLOW_PERIOD;
              }
              joint_S2.limiter_hit_period = JOINT_S2_LIMITER_HIT_PERIOD;
            }
          }
          else
          {
            joint_S2.state = JOINT_STATE_STEPPER_HOLDING;
            --active_joints_count;
          }
          joint_S2_pulsed = true;
          joint_S2.assumed_time += joint_S2.current_period;
        }
        break;
      }
      case JOINT_STATE_STEPPER_MOVING_AWAY_LIMITER:
      {
        if (++joint_S2.discrete_time - joint_S2.assumed_time > joint_S2.current_period)
        {
          if (joint_S2.motion_steps_remaining > 0)
          {
            digitalWrite(PIN_MOTO3_DIR, HIGH);
            digitalWrite(PIN_MOTO3_PUL, HIGH);
            --joint_S2.motion_steps_remaining;
            --joint_S2.executed_motion_steps;
          }
          else if (joint_S2.motion_steps_remaining < 0)
          {
            digitalWrite(PIN_MOTO3_DIR, LOW);
            digitalWrite(PIN_MOTO3_PUL, HIGH);
            ++joint_S2.motion_steps_remaining;
            ++joint_S2.executed_motion_steps;
          }
          else
          {
            joint_S2.state = JOINT_STATE_STEPPER_HOLDING;
            --active_joints_count;
          }
          joint_S2_pulsed = true;
          joint_S2.assumed_time += joint_S2.current_period;
        }
        break;
      }
      default:
      {
        break;
      }
  }


  switch (joint_E1.state)
  {
      case JOINT_STATE_STEPPER_HOLDING:
      {
        if (!command_Queue_E1.empty())
        {
          if (active_parallel_stage == command_Queue_E1.Commands[command_Queue_E1.first].parallel_stage)
          {
            joint_E1.motion_steps_remaining = command_Queue_E1.Commands[command_Queue_E1.first].motion_steps;
            joint_E1.current_period = JOINT_E1_TIMER_SLOW_PERIOD;
            joint_E1.state = JOINT_STATE_STEPPER_MOVING_FORWARD;

            joint_E1.discrete_time = 0;
            joint_E1.assumed_time = 0.0;
            joint_E1.total_steps_made = 0;

            command_Queue_E1.dequeue_Command();
            ++active_joints_count;

            main_state = MAIN_STATE_STEPPER_ACTIVE;
          }
        }
        break;
      }
      case JOINT_STATE_STEPPER_MOVING_FORWARD:
      {
        if (++joint_E1.discrete_time - joint_E1.assumed_time > joint_E1.current_period)
        {
          if (joint_E1.motion_steps_remaining > 0)
          {
            digitalWrite(PIN_MOTO5_DIR, HIGH);
            digitalWrite(PIN_MOTO5_PUL, HIGH);
            --joint_E1.motion_steps_remaining;
            --joint_E1.executed_motion_steps;
            ++joint_E1.total_steps_made;

            if (joint_E1_limiter_A == HIGH)
            {
              joint_E1.state = JOINT_STATE_STEPPER_HITTING_LIMITER;
              joint_E1.limiter_hit_period = JOINT_E1_LIMITER_HIT_PERIOD;
              joint_E1.limiter_hit_count = 1;
            }
          }
          else if (joint_E1.motion_steps_remaining < 0)
          {
            digitalWrite(PIN_MOTO5_DIR, LOW);
            digitalWrite(PIN_MOTO5_PUL, HIGH);
            ++joint_E1.motion_steps_remaining;
            ++joint_E1.executed_motion_steps;
            ++joint_E1.total_steps_made;

            if (joint_E1_limiter_B == HIGH)
            {
              joint_E1.state = JOINT_STATE_STEPPER_HITTING_LIMITER;
              joint_E1.limiter_hit_period = JOINT_E1_LIMITER_HIT_PERIOD;
              joint_E1.limiter_hit_count = 1;
            }
          }
          else
          {
            joint_E1.state = JOINT_STATE_STEPPER_HOLDING;
            --active_joints_count;
          }
          joint_E1_pulsed = true;
          joint_E1.assumed_time += joint_E1.current_period;
        }
        break;
      }
      case JOINT_STATE_STEPPER_HITTING_LIMITER:
      {
        if (++joint_E1.discrete_time - joint_E1.assumed_time > joint_E1.current_period)
        {
          if (--joint_E1.limiter_hit_period <= 0)
          {
            joint_E1.state = JOINT_STATE_STEPPER_MOVING_FORWARD;
          }

          if (joint_E1.motion_steps_remaining > 0)
          {
            digitalWrite(PIN_MOTO5_DIR, HIGH);
            digitalWrite(PIN_MOTO5_PUL, HIGH);
            --joint_E1.motion_steps_remaining;
            --joint_E1.executed_motion_steps;
            ++joint_E1.total_steps_made;

            if (joint_E1_limiter_A == HIGH)
            {
              if (++joint_E1.limiter_hit_count >= JOINT_E1_LIMITER_HIT_TRESHOLD)
              {
                joint_E1.state = JOINT_STATE_STEPPER_MOVING_AWAY_LIMITER;
                joint_E1.motion_steps_remaining = -JOINT_E1_AWAY_LIMITER_STEPS;
                joint_E1.current_period = JOINT_E1_TIMER_SLOW_PERIOD;
              }
              joint_E1.limiter_hit_period = JOINT_E1_LIMITER_HIT_PERIOD;
            }
          }
          else if (joint_E1.motion_steps_remaining < 0)
          {
            digitalWrite(PIN_MOTO5_DIR, LOW);
            digitalWrite(PIN_MOTO5_PUL, HIGH);
            ++joint_E1.motion_steps_remaining;
            ++joint_E1.executed_motion_steps;
            ++joint_E1.total_steps_made;

            if (joint_E1_limiter_B == HIGH)
            {
              if (++joint_E1.limiter_hit_count >= JOINT_E1_LIMITER_HIT_TRESHOLD)
              {
                joint_E1.state = JOINT_STATE_STEPPER_MOVING_AWAY_LIMITER;
                joint_E1.motion_steps_remaining = JOINT_E1_AWAY_LIMITER_STEPS;
                joint_E1.current_period = JOINT_E1_TIMER_SLOW_PERIOD;
              }
              joint_E1.limiter_hit_period = JOINT_E1_LIMITER_HIT_PERIOD;
            }
          }
          else
          {
            joint_E1.state = JOINT_STATE_STEPPER_HOLDING;
            --active_joints_count;
          }
          joint_E1_pulsed = true;
          joint_E1.assumed_time += joint_E1.current_period;
        }
        break;
      }
      case JOINT_STATE_STEPPER_MOVING_AWAY_LIMITER:
      {
        if (++joint_E1.discrete_time - joint_E1.assumed_time > joint_E1.current_period)
        {
          if (joint_E1.motion_steps_remaining > 0)
          {
            digitalWrite(PIN_MOTO5_DIR, HIGH);
            digitalWrite(PIN_MOTO5_PUL, HIGH);
            --joint_E1.motion_steps_remaining;
            --joint_E1.executed_motion_steps;
          }
          else if (joint_E1.motion_steps_remaining < 0)
          {
            digitalWrite(PIN_MOTO5_DIR, LOW);
            digitalWrite(PIN_MOTO5_PUL, HIGH);
            ++joint_E1.motion_steps_remaining;
            ++joint_E1.executed_motion_steps;
          }
          else
          {
            joint_E1.state = JOINT_STATE_STEPPER_HOLDING;
            --active_joints_count;
          }
          joint_E1_pulsed = true;
          joint_E1.assumed_time += joint_E1.current_period;
        }
        break;
      }
      default:
      {
        break;
      }
  }


  switch (joint_E2.state)
  {
      case JOINT_STATE_STEPPER_HOLDING:
      {
        if (!command_Queue_E2.empty())
        {
          if (active_parallel_stage == command_Queue_E2.Commands[command_Queue_E2.first].parallel_stage)
          {
            joint_E2.motion_steps_remaining = command_Queue_E2.Commands[command_Queue_E2.first].motion_steps;
            joint_E2.current_period = JOINT_E2_TIMER_SLOW_PERIOD;
            joint_E2.state = JOINT_STATE_STEPPER_MOVING_FORWARD;

            joint_E2.discrete_time = 0;
            joint_E2.assumed_time = 0.0;
            joint_E2.total_steps_made = 0;

            command_Queue_E2.dequeue_Command();
            ++active_joints_count;

            main_state = MAIN_STATE_STEPPER_ACTIVE;
          }
        }
        break;
      }
      case JOINT_STATE_STEPPER_MOVING_FORWARD:
      {
        if (++joint_E2.discrete_time - joint_E2.assumed_time > joint_E2.current_period)
        {
          if (joint_E2.motion_steps_remaining > 0)
          {
            digitalWrite(PIN_MOTO4_DIR, HIGH);
            digitalWrite(PIN_MOTO4_PUL, HIGH);
            --joint_E2.motion_steps_remaining;
            --joint_E2.executed_motion_steps;
            ++joint_E2.total_steps_made;

            if (joint_E2_limiter_A == HIGH)
            {
              joint_E2.state = JOINT_STATE_STEPPER_HITTING_LIMITER;
              joint_E2.limiter_hit_period = JOINT_E2_LIMITER_HIT_PERIOD;
              joint_E2.limiter_hit_count = 1;
            }
          }
          else if (joint_E2.motion_steps_remaining < 0)
          {
            digitalWrite(PIN_MOTO4_DIR, LOW);
            digitalWrite(PIN_MOTO4_PUL, HIGH);
            ++joint_E2.motion_steps_remaining;
            ++joint_E2.executed_motion_steps;
            ++joint_E2.total_steps_made;

            if (joint_E2_limiter_B == HIGH)
            {
              joint_E2.state = JOINT_STATE_STEPPER_HITTING_LIMITER;
              joint_E2.limiter_hit_period = JOINT_E2_LIMITER_HIT_PERIOD;
              joint_E2.limiter_hit_count = 1;
            }
          }
          else
          {
            joint_E2.state = JOINT_STATE_STEPPER_HOLDING;
            --active_joints_count;
          }
          joint_E2_pulsed = true;
          joint_E2.assumed_time += joint_E2.current_period;
        }
        break;
      }
      case JOINT_STATE_STEPPER_HITTING_LIMITER:
      {
        if (++joint_E2.discrete_time - joint_E2.assumed_time > joint_E2.current_period)
        {
          if (--joint_E2.limiter_hit_period <= 0)
          {
            joint_E2.state = JOINT_STATE_STEPPER_MOVING_FORWARD;
          }

          if (joint_E2.motion_steps_remaining > 0)
          {
            digitalWrite(PIN_MOTO4_DIR, HIGH);
            digitalWrite(PIN_MOTO4_PUL, HIGH);
            --joint_E2.motion_steps_remaining;
            --joint_E2.executed_motion_steps;
            ++joint_E2.total_steps_made;

            if (joint_E2_limiter_A == HIGH)
            {
              if (++joint_E2.limiter_hit_count >= JOINT_E2_LIMITER_HIT_TRESHOLD)
              {
                joint_E2.state = JOINT_STATE_STEPPER_MOVING_AWAY_LIMITER;
                joint_E2.motion_steps_remaining = -JOINT_E2_AWAY_LIMITER_STEPS;
                joint_E2.current_period = JOINT_E2_TIMER_SLOW_PERIOD;
              }
              joint_E2.limiter_hit_period = JOINT_E2_LIMITER_HIT_PERIOD;
            }
          }
          else if (joint_E2.motion_steps_remaining < 0)
          {
            digitalWrite(PIN_MOTO4_DIR, LOW);
            digitalWrite(PIN_MOTO4_PUL, HIGH);
            ++joint_E2.motion_steps_remaining;
            ++joint_E2.executed_motion_steps;
            ++joint_E2.total_steps_made;

            if (joint_E2_limiter_B == HIGH)
            {
              if (++joint_E2.limiter_hit_count >= JOINT_E2_LIMITER_HIT_TRESHOLD)
              {
                joint_E2.state = JOINT_STATE_STEPPER_MOVING_AWAY_LIMITER;
                joint_E2.motion_steps_remaining = JOINT_E2_AWAY_LIMITER_STEPS;
                joint_E2.current_period = JOINT_E2_TIMER_SLOW_PERIOD;
              }
              joint_E2.limiter_hit_period = JOINT_E2_LIMITER_HIT_PERIOD;
            }
          }
          else
          {
            joint_E2.state = JOINT_STATE_STEPPER_HOLDING;
            --active_joints_count;
          }
          joint_E2_pulsed = true;
          joint_E2.assumed_time += joint_E2.current_period;
        }
        break;
      }
      case JOINT_STATE_STEPPER_MOVING_AWAY_LIMITER:
      {
        if (++joint_E2.discrete_time - joint_E2.assumed_time > joint_E2.current_period)
        {
          if (joint_E2.motion_steps_remaining > 0)
          {
            digitalWrite(PIN_MOTO4_DIR, HIGH);
            digitalWrite(PIN_MOTO4_PUL, HIGH);
            --joint_E2.motion_steps_remaining;
            --joint_E2.executed_motion_steps;
          }
          else if (joint_E2.motion_steps_remaining < 0)
          {
            digitalWrite(PIN_MOTO4_DIR, LOW);
            digitalWrite(PIN_MOTO4_PUL, HIGH);
            ++joint_E2.motion_steps_remaining;
            ++joint_E2.executed_motion_steps;
          }
          else
          {
            joint_E2.state = JOINT_STATE_STEPPER_HOLDING;
            --active_joints_count;
          }
          joint_E2_pulsed = true;
          joint_E2.assumed_time += joint_E2.current_period;
        }
        break;
      }
      default:
      {
        break;
      }
  }


  switch (joint_W1.state)
  {
      case JOINT_STATE_STEPPER_HOLDING:
      {
        if (!command_Queue_W1.empty())
        {
          if (active_parallel_stage == command_Queue_W1.Commands[command_Queue_W1.first].parallel_stage)
          {
            joint_W1.motion_steps_remaining = command_Queue_W1.Commands[command_Queue_W1.first].motion_steps;
            joint_W1.current_period = JOINT_W1_TIMER_SLOW_PERIOD;
            joint_W1.state = JOINT_STATE_STEPPER_MOVING_FORWARD;

            joint_W1.discrete_time = 0;
            joint_W1.assumed_time = 0.0;
            joint_W1.total_steps_made = 0;

            command_Queue_W1.dequeue_Command();
            ++active_joints_count;

            main_state = MAIN_STATE_STEPPER_ACTIVE;
          }
        }
        break;
      }
      case JOINT_STATE_STEPPER_MOVING_FORWARD:
      {
        if (++joint_W1.discrete_time - joint_W1.assumed_time > joint_W1.current_period)
        {
          if (joint_W1.motion_steps_remaining > 0)
          {
            digitalWrite(PIN_MOTO2_DIR, HIGH);
            digitalWrite(PIN_MOTO2_PUL, HIGH);
            --joint_W1.motion_steps_remaining;
            --joint_W1.executed_motion_steps;
            ++joint_W1.total_steps_made;

            if (joint_W1_limiter_A == HIGH)
            {
              joint_W1.state = JOINT_STATE_STEPPER_HITTING_LIMITER;
              joint_W1.limiter_hit_period = JOINT_W1_LIMITER_HIT_PERIOD;
              joint_W1.limiter_hit_count = 1;              
            }
          }
          else if (joint_W1.motion_steps_remaining < 0)
          {
            digitalWrite(PIN_MOTO2_DIR, LOW);
            digitalWrite(PIN_MOTO2_PUL, HIGH);
            ++joint_W1.motion_steps_remaining;
            ++joint_W1.executed_motion_steps;
            ++joint_W1.total_steps_made;

            if (joint_W1_limiter_B == HIGH)
            {
              joint_W1.state = JOINT_STATE_STEPPER_HITTING_LIMITER;
              joint_W1.limiter_hit_period = JOINT_W1_LIMITER_HIT_PERIOD;
              joint_W1.limiter_hit_count = 1;
            }
          }
          else
          {
            joint_W1.state = JOINT_STATE_STEPPER_HOLDING;
            --active_joints_count;
          }
          joint_W1_pulsed = true;
          joint_W1.assumed_time += joint_W1.current_period;
        }
        break;
      }
      case JOINT_STATE_STEPPER_HITTING_LIMITER:
      {
        if (++joint_W1.discrete_time - joint_W1.assumed_time > joint_W1.current_period)
        {
          if (--joint_W1.limiter_hit_period <= 0)
          {
            joint_W1.state = JOINT_STATE_STEPPER_MOVING_FORWARD;
          }

          if (joint_W1.motion_steps_remaining > 0)
          {
            digitalWrite(PIN_MOTO2_DIR, HIGH);
            digitalWrite(PIN_MOTO2_PUL, HIGH);
            --joint_W1.motion_steps_remaining;
            --joint_W1.executed_motion_steps;
            ++joint_W1.total_steps_made;

            if (joint_W1_limiter_A == HIGH)
            {
              if (++joint_W1.limiter_hit_count >= JOINT_W1_LIMITER_HIT_TRESHOLD)
              {
                joint_W1.state = JOINT_STATE_STEPPER_MOVING_AWAY_LIMITER;
                joint_W1.motion_steps_remaining = -JOINT_W1_AWAY_LIMITER_STEPS;
                joint_W1.current_period = JOINT_W1_TIMER_SLOW_PERIOD;
              }
              joint_W1.limiter_hit_period = JOINT_W1_LIMITER_HIT_PERIOD;
            }
          }
          else if (joint_W1.motion_steps_remaining < 0)
          {
            digitalWrite(PIN_MOTO2_DIR, LOW);
            digitalWrite(PIN_MOTO2_PUL, HIGH);
            ++joint_W1.motion_steps_remaining;
            ++joint_W1.executed_motion_steps;
            ++joint_W1.total_steps_made;

            if (joint_W1_limiter_B == HIGH)
            {
              if (++joint_W1.limiter_hit_count >= JOINT_W1_LIMITER_HIT_TRESHOLD)
              {
                joint_W1.state = JOINT_STATE_STEPPER_MOVING_AWAY_LIMITER;
                joint_W1.motion_steps_remaining = JOINT_W1_AWAY_LIMITER_STEPS;
                joint_W1.current_period = JOINT_W1_TIMER_SLOW_PERIOD;
              }
              joint_W1.limiter_hit_period = JOINT_W1_LIMITER_HIT_PERIOD;
            }
          }
          else
          {
            joint_W1.state = JOINT_STATE_STEPPER_HOLDING;
            --active_joints_count;
          }
          joint_W1_pulsed = true;
          joint_W1.assumed_time += joint_W1.current_period;
        }
        break;
      }
      case JOINT_STATE_STEPPER_MOVING_AWAY_LIMITER:
      {
        if (++joint_W1.discrete_time - joint_W1.assumed_time > joint_W1.current_period)
        {
          if (joint_W1.motion_steps_remaining > 0)
          {
            digitalWrite(PIN_MOTO2_DIR, HIGH);
            digitalWrite(PIN_MOTO2_PUL, HIGH);
            --joint_W1.motion_steps_remaining;
            --joint_W1.executed_motion_steps;
          }
          else if (joint_W1.motion_steps_remaining < 0)
          {
            digitalWrite(PIN_MOTO2_DIR, LOW);
            digitalWrite(PIN_MOTO2_PUL, HIGH);
            ++joint_W1.motion_steps_remaining;
            ++joint_W1.executed_motion_steps;
          }
          else
          {
            joint_W1.state = JOINT_STATE_STEPPER_HOLDING;
            --active_joints_count;
          }
          joint_W1_pulsed = true;
          joint_W1.assumed_time += joint_W1.current_period;
        }
        break;
      }
      default:
      {
        break;
      }
  }


  switch (joint_W2.state)
  {
      case JOINT_STATE_STEPPER_HOLDING:
      {
        if (!command_Queue_W2.empty())
        {
          if (active_parallel_stage == command_Queue_W2.Commands[command_Queue_W2.first].parallel_stage)
          {
            joint_W2.motion_steps_remaining = command_Queue_W2.Commands[command_Queue_W2.first].motion_steps;
            joint_W2.current_period = JOINT_W2_TIMER_SLOW_PERIOD;
            joint_W2.state = JOINT_STATE_STEPPER_MOVING_FORWARD;

            joint_W2.discrete_time = 0;
            joint_W2.assumed_time = 0.0;
            joint_W2.total_steps_made = 0;

            command_Queue_W2.dequeue_Command();
            ++active_joints_count;

            main_state = MAIN_STATE_STEPPER_ACTIVE;
          }
        }
        break;
      }
      case JOINT_STATE_STEPPER_MOVING_FORWARD:
      {
        if (++joint_W2.discrete_time - joint_W2.assumed_time > joint_W2.current_period)
        {
          if (joint_W2.motion_steps_remaining > 0)
          {
            digitalWrite(PIN_MOTO7_DIR, HIGH);
            digitalWrite(PIN_MOTO7_PUL, HIGH);
            --joint_W2.motion_steps_remaining;
            --joint_W2.executed_motion_steps;
            ++joint_W2.total_steps_made;

            if (joint_W2_limiter_A == HIGH)
            {
              joint_W2.state = JOINT_STATE_STEPPER_HITTING_LIMITER;
              joint_W2.limiter_hit_period = JOINT_W2_LIMITER_HIT_PERIOD;
              joint_W2.limiter_hit_count = 1;
            }
          }
          else if (joint_W2.motion_steps_remaining < 0)
          {
            digitalWrite(PIN_MOTO7_DIR, LOW);
            digitalWrite(PIN_MOTO7_PUL, HIGH);
            ++joint_W2.motion_steps_remaining;
            ++joint_W2.executed_motion_steps;
            ++joint_W2.total_steps_made;

            if (joint_W2_limiter_B == HIGH)
            {
              joint_W2.state = JOINT_STATE_STEPPER_HITTING_LIMITER;
              joint_W2.limiter_hit_period = JOINT_W2_LIMITER_HIT_PERIOD;
              joint_W2.limiter_hit_count = 1;
            }
          }
          else
          {
            joint_W2.state = JOINT_STATE_STEPPER_HOLDING;
            --active_joints_count;
          }
          joint_W2_pulsed = true;
          joint_W2.assumed_time += joint_W2.current_period;
        }
        break;
      }
      case JOINT_STATE_STEPPER_HITTING_LIMITER:
      {
        if (++joint_W2.discrete_time - joint_W2.assumed_time > joint_W2.current_period)
        {
          if (--joint_W2.limiter_hit_period <= 0)
          {
            joint_W2.state = JOINT_STATE_STEPPER_MOVING_FORWARD;
          }

          if (joint_W2.motion_steps_remaining > 0)
          {
            digitalWrite(PIN_MOTO7_DIR, HIGH);
            digitalWrite(PIN_MOTO7_PUL, HIGH);
            --joint_W2.motion_steps_remaining;
            --joint_W2.executed_motion_steps;
            ++joint_W2.total_steps_made;

            if (joint_W2_limiter_A == HIGH)
            {
              if (++joint_W2.limiter_hit_count >= JOINT_W2_LIMITER_HIT_TRESHOLD)
              {
                joint_W2.state = JOINT_STATE_STEPPER_MOVING_AWAY_LIMITER;
                joint_W2.motion_steps_remaining = -JOINT_W2_AWAY_LIMITER_STEPS;
                joint_W2.current_period = JOINT_W2_TIMER_SLOW_PERIOD;
              }
              joint_W2.limiter_hit_period = JOINT_W2_LIMITER_HIT_PERIOD;
            }
          }
          else if (joint_W2.motion_steps_remaining < 0)
          {
            digitalWrite(PIN_MOTO7_DIR, LOW);
            digitalWrite(PIN_MOTO7_PUL, HIGH);
            ++joint_W2.motion_steps_remaining;
            ++joint_W2.executed_motion_steps;
            ++joint_W2.total_steps_made;

            if (joint_W2_limiter_B == HIGH)
            {
              if (++joint_W2.limiter_hit_count >= JOINT_W2_LIMITER_HIT_TRESHOLD)
              {
                joint_W2.state = JOINT_STATE_STEPPER_MOVING_AWAY_LIMITER;
                joint_W2.motion_steps_remaining = JOINT_W2_AWAY_LIMITER_STEPS;
                joint_W2.current_period = JOINT_W2_TIMER_SLOW_PERIOD;
              }
              joint_W2.limiter_hit_period = JOINT_W2_LIMITER_HIT_PERIOD;
            }
          }
          else
          {
            joint_W2.state = JOINT_STATE_STEPPER_HOLDING;
            --active_joints_count;
          }
          joint_W2_pulsed = true;
          joint_W2.assumed_time += joint_W2.current_period;
        }
        break;
      }
      case JOINT_STATE_STEPPER_MOVING_AWAY_LIMITER:
      {
        if (++joint_W2.discrete_time - joint_W2.assumed_time > joint_W2.current_period)
        {
          if (joint_W2.motion_steps_remaining > 0)
          {
            digitalWrite(PIN_MOTO7_DIR, HIGH);
            digitalWrite(PIN_MOTO7_PUL, HIGH);
            --joint_W2.motion_steps_remaining;
            --joint_W2.executed_motion_steps;
          }
          else if (joint_W2.motion_steps_remaining < 0)
          {
            digitalWrite(PIN_MOTO7_DIR, LOW);
            digitalWrite(PIN_MOTO7_PUL, HIGH);            
            ++joint_W2.motion_steps_remaining;
            ++joint_W2.executed_motion_steps;
          }
          else
          {
            joint_W2.state = JOINT_STATE_STEPPER_HOLDING;
            --active_joints_count;
          }
          joint_W2_pulsed = true;
          joint_W2.assumed_time += joint_W2.current_period;
        }
        break;
      }
      default:
      {
        break;
      }
  }

  switch (joint_G.state)
  {
      case JOINT_STATE_STEPPER_HOLDING:
      {
        if (!command_Queue_G.empty())
        {
          if (active_parallel_stage == command_Queue_G.Commands[command_Queue_G.first].parallel_stage)
          {
            joint_G.motion_steps_remaining = command_Queue_G.Commands[command_Queue_G.first].motion_steps;
            joint_G.current_period = JOINT_G_TIMER_SLOW_PERIOD;
            joint_G.state = JOINT_STATE_STEPPER_MOVING_FORWARD;

            joint_G.discrete_time = 0;
            joint_G.assumed_time = 0.0;
            joint_G.total_steps_made = 0;

            command_Queue_G.dequeue_Command();
            ++active_joints_count;

            main_state = MAIN_STATE_STEPPER_ACTIVE;
          }
        }
        break;
      }
      case JOINT_STATE_STEPPER_MOVING_FORWARD:
      {
        if (++joint_G.discrete_time - joint_G.assumed_time > joint_G.current_period)
        {
          if (joint_G.motion_steps_remaining > 0)
          {
            digitalWrite(PIN_MOTO1_DIR, HIGH);
            digitalWrite(PIN_MOTO1_PUL, HIGH);
            --joint_G.motion_steps_remaining;
            --joint_G.executed_motion_steps;
            ++joint_G.total_steps_made;

            if (joint_G_limiter_A == HIGH)
            {
              joint_G.state = JOINT_STATE_STEPPER_HITTING_LIMITER;
              joint_G.limiter_hit_period = JOINT_G_LIMITER_HIT_PERIOD;
              joint_G.limiter_hit_count = 1;
            }
          }
          else if (joint_G.motion_steps_remaining < 0)
          {
            digitalWrite(PIN_MOTO1_DIR, LOW);
            digitalWrite(PIN_MOTO1_PUL, HIGH);
            ++joint_G.motion_steps_remaining;
            ++joint_G.executed_motion_steps;
            ++joint_G.total_steps_made;

            if (joint_G_limiter_B == HIGH)
            {
              joint_G.state = JOINT_STATE_STEPPER_HITTING_LIMITER;
              joint_G.limiter_hit_period = JOINT_G_LIMITER_HIT_PERIOD;
              joint_G.limiter_hit_count = 1;
            }
          }
          else
          {
            joint_G.state = JOINT_STATE_STEPPER_HOLDING;
            --active_joints_count;
          }
          joint_G_pulsed = true;
          joint_G.assumed_time += joint_G.current_period;
        }
        break;
      }
      case JOINT_STATE_STEPPER_HITTING_LIMITER:
      {
        if (++joint_G.discrete_time - joint_G.assumed_time > joint_G.current_period)
        {
          if (--joint_G.limiter_hit_period <= 0)
          {
            joint_G.state = JOINT_STATE_STEPPER_MOVING_FORWARD;
          }

          if (joint_G.motion_steps_remaining > 0)
          {
            digitalWrite(PIN_MOTO1_DIR, HIGH);
            digitalWrite(PIN_MOTO1_PUL, HIGH);
            --joint_G.motion_steps_remaining;
            --joint_G.executed_motion_steps;
            ++joint_G.total_steps_made;

            if (joint_G_limiter_A == HIGH)
            {
              if (++joint_G.limiter_hit_count >= JOINT_G_LIMITER_HIT_TRESHOLD)
              {
                joint_G.state = JOINT_STATE_STEPPER_MOVING_AWAY_LIMITER;
                joint_G.motion_steps_remaining = -JOINT_G_AWAY_LIMITER_STEPS;
                joint_G.current_period = JOINT_G_TIMER_SLOW_PERIOD;
              }
              joint_G.limiter_hit_period = JOINT_G_LIMITER_HIT_PERIOD;
            }
          }
          else if (joint_G.motion_steps_remaining < 0)
          {
            digitalWrite(PIN_MOTO1_DIR, LOW);
            digitalWrite(PIN_MOTO1_PUL, HIGH);
            ++joint_G.motion_steps_remaining;
            ++joint_G.executed_motion_steps;
            ++joint_G.total_steps_made;

            if (joint_G_limiter_B == HIGH)
            {
              if (++joint_G.limiter_hit_count >= JOINT_G_LIMITER_HIT_TRESHOLD)
              {
                joint_G.state = JOINT_STATE_STEPPER_MOVING_AWAY_LIMITER;
                joint_G.motion_steps_remaining = JOINT_G_AWAY_LIMITER_STEPS;
                joint_G.current_period = JOINT_G_TIMER_SLOW_PERIOD;
              }
              joint_G.limiter_hit_period = JOINT_G_LIMITER_HIT_PERIOD;
            }
          }
          else
          {
            joint_G.state = JOINT_STATE_STEPPER_HOLDING;
            --active_joints_count;
          }
          joint_G_pulsed = true;
          joint_G.assumed_time += joint_G.current_period;
        }
        break;
      }
      case JOINT_STATE_STEPPER_MOVING_AWAY_LIMITER:
      {
        if (++joint_G.discrete_time - joint_G.assumed_time > joint_G.current_period)
        {
          if (joint_G.motion_steps_remaining > 0)
          {
            digitalWrite(PIN_MOTO1_DIR, HIGH);
            digitalWrite(PIN_MOTO1_PUL, HIGH);
            --joint_G.motion_steps_remaining;
            --joint_G.executed_motion_steps;            
          }
          else if (joint_G.motion_steps_remaining < 0)
          {
            digitalWrite(PIN_MOTO1_DIR, LOW);
            digitalWrite(PIN_MOTO1_PUL, HIGH);
            ++joint_G.motion_steps_remaining;
            ++joint_G.executed_motion_steps;
          }
          else
          {
            joint_G.state = JOINT_STATE_STEPPER_HOLDING;
            --active_joints_count;
          }
          joint_G_pulsed = true;
          joint_G.assumed_time += joint_G.current_period;
        }
        break;
      }
      default:
      {
        break;
      }
  }
  /*
    Serial.print("Koko:");
    Serial.print(amt_encoder1.encoder.koko);
    Serial.print(",");
    Serial.print(amt_encoder2.encoder.koko);
    Serial.print(",");
    Serial.print(amt_encoder3.encoder.koko);
    Serial.print(",");
    Serial.print(amt_encoder4.encoder.koko);
    Serial.print(",");
    Serial.print(amt_encoder5.encoder.koko);
    Serial.print(",");
    Serial.print(amt_encoder6.encoder.koko);
    Serial.print(",");
    Serial.print(amt_encoder7.encoder.koko);
    Serial.print("\n");
  */

  int position_J_G = amt_encoder1.read();
  int position_J_W1 = amt_encoder2.read();
  int position_J_S2 = amt_encoder3.read();
  int position_J_E2 = amt_encoder4.read();
  int position_J_E1 = amt_encoder5.read();
  int position_J_S1 = amt_encoder6.read();
  int position_J_W2 = amt_encoder7.read();

  /*
     Serial.print("Positions:");
     Serial.print(position_J_S1);
     Serial.print(" ");
     Serial.print(position_J_S2);
     Serial.print(" ");
     Serial.print(position_J_E1);
     Serial.print(" ");
     Serial.print(position_J_E2);
     Serial.print(" ");
     Serial.print(position_J_W1);
     Serial.print(" ");
     Serial.print(position_J_W2);
     Serial.print(" ");
     Serial.print(position_J_G);
     Serial.print(" ");
     Serial.print("\n");
  */


  /*
     int val_PIN_LIMITER1_A = digitalRead(PIN_LIMITER1_A);
     int val_PIN_LIMITER1_B = digitalRead(PIN_LIMITER1_B);
     int val_PIN_LIMITER2_A = digitalRead(PIN_LIMITER2_A);
     int val_PIN_LIMITER2_B = digitalRead(PIN_LIMITER2_B);
     int val_PIN_LIMITER3_A = digitalRead(PIN_LIMITER3_A);
     int val_PIN_LIMITER3_B = digitalRead(PIN_LIMITER3_B);
     int val_PIN_LIMITER4_A = digitalRead(PIN_LIMITER4_A);
     int val_PIN_LIMITER4_B = digitalRead(PIN_LIMITER4_B);
     int val_PIN_LIMITER5_A = digitalRead(PIN_LIMITER5_A);
     int val_PIN_LIMITER5_B = digitalRead(PIN_LIMITER5_B);
     int val_PIN_LIMITER6_A = digitalRead(PIN_LIMITER6_A);
     int val_PIN_LIMITER6_B = digitalRead(PIN_LIMITER6_B);
     int val_PIN_LIMITER7_A = digitalRead(PIN_LIMITER7_A);
     int val_PIN_LIMITER7_B = digitalRead(PIN_LIMITER7_B);
     int val_stop     = digitalRead(PIN_EMERGENCY_STOP);


     Serial.print("PIN_LIMITERy:");

     Serial.print("1:");
     Serial.print(val_PIN_LIMITER1_A);
     Serial.print(" ");
     Serial.print(val_PIN_LIMITER1_B);
     Serial.print("    ");

     Serial.print("2:");
     Serial.print(val_PIN_LIMITER2_A);
     Serial.print(" ");
     Serial.print(val_PIN_LIMITER2_B);
     Serial.print("    ");

     Serial.print("3:");
     Serial.print(val_PIN_LIMITER3_A);
     Serial.print(" ");
     Serial.print(val_PIN_LIMITER3_B);
     Serial.print("    ");

     Serial.print("4:");
     Serial.print(val_PIN_LIMITER4_A);
     Serial.print(" ");
     Serial.print(val_PIN_LIMITER4_B);
     Serial.print("    ");

     Serial.print("5:");
     Serial.print(val_PIN_LIMITER5_A);
     Serial.print(" ");
     Serial.print(val_PIN_LIMITER5_B);
     Serial.print("    ");

     Serial.print("6:");
     Serial.print(val_PIN_LIMITER6_A);
     Serial.print(" ");
     Serial.print(val_PIN_LIMITER6_B);
     Serial.print("    ");

     Serial.print("7:");
     Serial.print(val_PIN_LIMITER7_A);
     Serial.print(" ");
     Serial.print(val_PIN_LIMITER7_B);
     Serial.print("    ");

     Serial.print("\n");

     Serial.print("Emergency stop:");
     Serial.print(val_stop);
     Serial.print("\n");
  */
  delayMicroseconds(MAIN_PULSE_DELAY);

  switch (joint_S1.state)
  {
      case JOINT_STATE_STEPPER_MOVING_FORWARD:
      case JOINT_STATE_STEPPER_HITTING_LIMITER:
      {
        if (joint_S1.discrete_time <= joint_S1.time_to_speed)
        {
          joint_S1.current_period = calc_Speed2Period(calc_CurrentSpeed(joint_S1.discrete_time, joint_S1.time_to_speed, joint_S1.calc_FinalSpeed()));
        }
        else
        {
          if (joint_S1.discrete_time >= joint_S1.total_motion_time - joint_S1.time_to_speed)
          {
            joint_S1.current_period = calc_Speed2Period(calc_CurrentSpeed(joint_S1.total_motion_time - joint_S1.discrete_time, joint_S1.time_to_speed, joint_S1.calc_FinalSpeed()));
          }
        }                
        if (joint_S1_pulsed)
        {
          digitalWrite(PIN_MOTO6_PUL, LOW);
          joint_S1_pulsed = false;
        }
        break;
      }
      case JOINT_STATE_STEPPER_MOVING_AWAY_LIMITER:
      {
        if (joint_S1_pulsed)
        {
          digitalWrite(PIN_MOTO6_PUL, LOW);
          joint_S1_pulsed = false;
        }
        break;
      }
      default:
      {
        break;
      }
  }

  switch (joint_S2.state)
  {
      case JOINT_STATE_STEPPER_MOVING_FORWARD:
      case JOINT_STATE_STEPPER_HITTING_LIMITER:
      {       
        if (joint_S2.discrete_time <= joint_S2.time_to_speed)
        {
          joint_S2.current_period = calc_Speed2Period(calc_CurrentSpeed(joint_S2.discrete_time, joint_S2.time_to_speed, joint_S2.calc_FinalSpeed()));
        }
        else
        {
          if (joint_S2.discrete_time >= joint_S2.total_motion_time - joint_S2.time_to_speed)
          {
            joint_S2.current_period = calc_Speed2Period(calc_CurrentSpeed(joint_S2.total_motion_time - joint_S2.discrete_time, joint_S2.time_to_speed, joint_S2.calc_FinalSpeed()));
          }
        }                        
        if (joint_S2_pulsed)
        {
          digitalWrite(PIN_MOTO3_PUL, LOW);
          joint_S2_pulsed = false;
        }
        break;
      }
      case JOINT_STATE_STEPPER_MOVING_AWAY_LIMITER:
      {
        if (joint_S2_pulsed)
        {
          digitalWrite(PIN_MOTO3_PUL, LOW);
          joint_S2_pulsed = false;
        }
        break;
      }
      default:
      {
        break;
      }
  }


  switch (joint_E1.state)
  {
      case JOINT_STATE_STEPPER_MOVING_FORWARD:
      case JOINT_STATE_STEPPER_HITTING_LIMITER:
      {
        if (joint_E1.discrete_time <= joint_E1.time_to_speed)
        {
          joint_E1.current_period = calc_Speed2Period(calc_CurrentSpeed(joint_E1.discrete_time, joint_E1.time_to_speed, joint_E1.calc_FinalSpeed()));
        }
        else
        {
          if (joint_E1.discrete_time >= joint_E1.total_motion_time - joint_E1.time_to_speed)
          {
            joint_E1.current_period = calc_Speed2Period(calc_CurrentSpeed(joint_E1.total_motion_time - joint_E1.discrete_time, joint_E1.time_to_speed, joint_E1.calc_FinalSpeed()));
          }
        }                                
        if (joint_E1_pulsed)
        {
          digitalWrite(PIN_MOTO5_PUL, LOW);
          joint_E1_pulsed = false;
        }
        break;
      }
      case JOINT_STATE_STEPPER_MOVING_AWAY_LIMITER:
      {
        if (joint_E1_pulsed)
        {
          digitalWrite(PIN_MOTO5_PUL, LOW);
          joint_E1_pulsed = false;
        }
        break;
      }
      default:
      {
        break;
      }
  }


  switch (joint_E2.state)
  {
      case JOINT_STATE_STEPPER_MOVING_FORWARD:
      case JOINT_STATE_STEPPER_HITTING_LIMITER:
      {
        if (joint_E2.discrete_time <= joint_E2.time_to_speed)
        {
          joint_E2.current_period = calc_Speed2Period(calc_CurrentSpeed(joint_E2.discrete_time, joint_E2.time_to_speed, joint_E2.calc_FinalSpeed()));
        }
        else
        {
          if (joint_E2.discrete_time >= joint_E2.total_motion_time - joint_E2.time_to_speed)
          {
            joint_E2.current_period = calc_Speed2Period(calc_CurrentSpeed(joint_E2.total_motion_time - joint_E2.discrete_time, joint_E2.time_to_speed, joint_E2.calc_FinalSpeed()));
          }
        }                                        
        if (joint_E2_pulsed)
        {
          digitalWrite(PIN_MOTO4_PUL, LOW);
          joint_E2_pulsed = false;
        }
        break;
      }
      case JOINT_STATE_STEPPER_MOVING_AWAY_LIMITER:
      {
        if (joint_E2_pulsed)
        {
          digitalWrite(PIN_MOTO4_PUL, LOW);
          joint_E2_pulsed = false;
        }
        break;
      }
      default:
      {
        break;
      }
  }


  switch (joint_W1.state)
  {
      case JOINT_STATE_STEPPER_MOVING_FORWARD:
      case JOINT_STATE_STEPPER_HITTING_LIMITER:
      {
        if (joint_W1.discrete_time <= joint_W1.time_to_speed)
        {
          joint_W1.current_period = calc_Speed2Period(calc_CurrentSpeed(joint_W1.discrete_time, joint_W1.time_to_speed, joint_W1.calc_FinalSpeed()));
        }
        else
        {
          if (joint_W1.discrete_time >= joint_W1.total_motion_time - joint_W1.time_to_speed)
          {
            joint_W1.current_period = calc_Speed2Period(calc_CurrentSpeed(joint_W1.total_motion_time - joint_W1.discrete_time, joint_W1.time_to_speed, joint_W1.calc_FinalSpeed()));
          }
        }                
        if (joint_W1_pulsed)
        {
          digitalWrite(PIN_MOTO2_PUL, LOW);
          joint_W1_pulsed = false;
        }
        break;
      }
      case JOINT_STATE_STEPPER_MOVING_AWAY_LIMITER:
      {
        if (joint_W1_pulsed)
        {
          digitalWrite(PIN_MOTO2_PUL, LOW);
          joint_W1_pulsed = false;
        }
        break;
      }
      default:
      {
        break;
      }
  }

  switch (joint_W2.state)
  {
      case JOINT_STATE_STEPPER_MOVING_FORWARD:
      case JOINT_STATE_STEPPER_HITTING_LIMITER:
      {
        if (joint_W2.discrete_time <= joint_W2.time_to_speed)
        {
          joint_W2.current_period = calc_Speed2Period(calc_CurrentSpeed(joint_W2.discrete_time, joint_W2.time_to_speed, joint_W2.calc_FinalSpeed()));
        }
        else
        {
          if (joint_W2.discrete_time >= joint_W2.total_motion_time - joint_W2.time_to_speed)
          {
            joint_W2.current_period = calc_Speed2Period(calc_CurrentSpeed(joint_W2.total_motion_time - joint_W2.discrete_time, joint_W2.time_to_speed, joint_W2.calc_FinalSpeed()));
          }
        }                        
        if (joint_W2_pulsed)
        {
          digitalWrite(PIN_MOTO7_PUL, LOW);
          joint_W2_pulsed = false;
        }
        break;
      }
      case JOINT_STATE_STEPPER_MOVING_AWAY_LIMITER:
      {
        if (joint_W2_pulsed)
        {
          digitalWrite(PIN_MOTO7_PUL, LOW);
          joint_W2_pulsed = false;
        }
        break;
      }
      default:
      {
        break;
      }
  }

  switch (joint_G.state)
  {
      case JOINT_STATE_STEPPER_MOVING_FORWARD:
      case JOINT_STATE_STEPPER_HITTING_LIMITER:
      {
        if (joint_G.discrete_time <= joint_G.time_to_speed)
        {
          joint_G.current_period = calc_Speed2Period(calc_CurrentSpeed(joint_G.discrete_time, joint_G.time_to_speed, joint_G.calc_FinalSpeed()));
        }
        else
        {
          if (joint_G.discrete_time >= joint_G.total_motion_time - joint_G.time_to_speed)
          {
            joint_G.current_period = calc_Speed2Period(calc_CurrentSpeed(joint_G.total_motion_time - joint_G.discrete_time, joint_G.time_to_speed, joint_G.calc_FinalSpeed()));
          }
        }                                
        if (joint_G_pulsed)
        {
          digitalWrite(PIN_MOTO1_PUL, LOW);
          joint_G_pulsed = false;
        }
        break;
      }
      case JOINT_STATE_STEPPER_MOVING_AWAY_LIMITER:
      {
        if (joint_G_pulsed)
        {
          digitalWrite(PIN_MOTO1_PUL, LOW);
          joint_G_pulsed = false;
        }
        break;
      }
      default:
      {
        break;
      }
  }

  delayMicroseconds(MAIN_PULSE_DELAY);

  if (--serial_communication_timeout <= 0)
  {
    Serial.write(RR_robot_header);
    Serial.write(RR1_SERIAL_NUMBER);

    Serial.write((const char*)&main_state, sizeof(main_state));
    Serial.write((const char*)&joints_limiters_state, sizeof(joints_limiters_state));
    joints_limiters_state = 0;

    Serial.write((const char*)&position_J_S1, sizeof(position_J_S1));
    Serial.write((const char*)&position_J_S2, sizeof(position_J_S2));
    Serial.write((const char*)&position_J_E1, sizeof(position_J_E1));
    Serial.write((const char*)&position_J_E2, sizeof(position_J_E2));
    Serial.write((const char*)&position_J_W1, sizeof(position_J_W1));
    Serial.write((const char*)&position_J_W2, sizeof(position_J_W2));
    Serial.write((const char*)&position_J_G, sizeof(position_J_G));

    Serial.write((const char*)&joint_S1.motion_steps_remaining, sizeof(joint_S1.motion_steps_remaining));
    Serial.write((const char*)&joint_S2.motion_steps_remaining, sizeof(joint_S2.motion_steps_remaining));
    Serial.write((const char*)&joint_E1.motion_steps_remaining, sizeof(joint_E1.motion_steps_remaining));
    Serial.write((const char*)&joint_E2.motion_steps_remaining, sizeof(joint_E2.motion_steps_remaining));
    Serial.write((const char*)&joint_W1.motion_steps_remaining, sizeof(joint_W1.motion_steps_remaining));
    Serial.write((const char*)&joint_W2.motion_steps_remaining, sizeof(joint_W2.motion_steps_remaining));
    Serial.write((const char*)&joint_G.motion_steps_remaining, sizeof(joint_G.motion_steps_remaining));

    Serial.write((const char*)&joint_S1.executed_motion_steps, sizeof(joint_S1.executed_motion_steps));
    Serial.write((const char*)&joint_S2.executed_motion_steps, sizeof(joint_S2.executed_motion_steps));
    Serial.write((const char*)&joint_E1.executed_motion_steps, sizeof(joint_E1.executed_motion_steps));
    Serial.write((const char*)&joint_E2.executed_motion_steps, sizeof(joint_E2.executed_motion_steps));
    Serial.write((const char*)&joint_W1.executed_motion_steps, sizeof(joint_W1.executed_motion_steps));
    Serial.write((const char*)&joint_W2.executed_motion_steps, sizeof(joint_W2.executed_motion_steps));
    Serial.write((const char*)&joint_G.executed_motion_steps, sizeof(joint_G.executed_motion_steps));    
    
    int available = Serial.available();

    if (available > 0)
    {
      int n_rd = Serial.readBytes(input_buffer + input_rd, available);
      input_rd += n_rd;

      /*
      Serial.print("gotto:");
      for (int i = 0; i < n_rd; ++i)
      {
        Serial.write(input_buffer[i]);
      }
      */
      
      int expected_size = sizeof(RR_robot_header) - 1 + 8 * sizeof(int);

      if (input_rd >= expected_size)
      {
        for (int i = 0; i < input_rd - sizeof(RR_robot_header); ++i)
        {
          bool match = true;
          for (int j = 0; j < sizeof(RR_robot_header) - 1; ++j)
          {
            if (input_buffer[i + j] != RR_robot_header[j])
            {
              match = false;
              break;
            }
          }

          if (match && i == 0)
          {
            char *joints_buffer = input_buffer + sizeof(RR_robot_header) - 1;

            int interactive_stepper_safety = *((int*)(joints_buffer + 0 * sizeof(int)));            

            int kbhit_J_S1_steps = *((int*)(joints_buffer + 1 * sizeof(int)));
            int kbhit_J_S2_steps = *((int*)(joints_buffer + 2 * sizeof(int)));
            int kbhit_J_E1_steps = *((int*)(joints_buffer + 3 * sizeof(int)));
            int kbhit_J_E2_steps = *((int*)(joints_buffer + 4 * sizeof(int)));
            int kbhit_J_W1_steps = *((int*)(joints_buffer + 5 * sizeof(int)));
            int kbhit_J_W2_steps = *((int*)(joints_buffer + 6 * sizeof(int)));
            int kbhit_J_G_steps  = *((int*)(joints_buffer + 7 * sizeof(int)));

            /*
            Serial.print("gotto:");
            Serial.print(kbhit_J_S1_steps);
            Serial.print("\n");
            Serial.print(kbhit_J_S2_steps);
            Serial.print("\n");            
            Serial.print(kbhit_J_E1_steps);
            Serial.print("\n");                        
            Serial.print(kbhit_J_E2_steps);
            Serial.print("\n");                                    
            Serial.print(kbhit_J_W1_steps);
            Serial.print("\n");                                                
            Serial.print(kbhit_J_W2_steps);
            Serial.print("\n");                                                            
            Serial.print(kbhit_J_G_steps);
            Serial.print("\n");                                                                        
            Serial.print(kbhit_J_S1_steps);
            Serial.print("endo\n");            
            */

            joint_S1.reset_MotionTimeEqualization();
            joint_S2.reset_MotionTimeEqualization();            
            joint_E1.reset_MotionTimeEqualization();
            joint_E2.reset_MotionTimeEqualization();                        
            joint_W1.reset_MotionTimeEqualization();
            joint_W2.reset_MotionTimeEqualization();                                    
            joint_G.reset_MotionTimeEqualization();

            int common_total_motion_time = 0;

            if (    abs(kbhit_J_S1_steps) > 0
                && (interactive_stepper_safety == INTERACTIVE_STEPPER_SAFETY_LOW || (interactive_stepper_safety == INTERACTIVE_STEPPER_SAFETY_HIGH && abs(kbhit_J_S1_steps) <= interactive_joint_step_limit)))
            {
              switch (joint_S1.state)
              {
                case JOINT_STATE_STEPPER_HOLDING:
                {
                  joint_S1.motion_steps_remaining += kbhit_J_S1_steps;
                  joint_S1.current_period = JOINT_S1_TIMER_STOP_PERIOD;
                  joint_S1.state = JOINT_STATE_STEPPER_MOVING_FORWARD;

                  joint_S1.steps_to_speed = calc_SmoothStepsToSpeed(abs(joint_S1.motion_steps_remaining), JOINT_S1_FAST_SPEED, JOINT_S1_STEPS_TO_FAST_SPEED, joint_S1.final_speed);
                  joint_S1.total_motion_time = calc_SmoothMotionTimes(joint_S1.final_speed, abs(joint_S1.motion_steps_remaining), joint_S1.steps_to_speed, joint_S1.time_to_speed);
                  common_total_motion_time = max(joint_S1.total_motion_time, common_total_motion_time);

                  joint_S1.discrete_time = 0;
                  joint_S1.assumed_time = 0.0;
                  joint_S1.total_steps_made = 0;

                  ++active_joints_count;
                  main_state = MAIN_STATE_STEPPER_ACTIVE;
                  break;
                }
                case JOINT_STATE_STEPPER_MOVING_FORWARD:
                {
                  if ((joint_S1.motion_steps_remaining > 0 && kbhit_J_S1_steps > 0) || (joint_S1.motion_steps_remaining < 0 && kbhit_J_S1_steps < 0))
                  {
                    if (joint_S1.discrete_time < joint_S1.total_motion_time - joint_S1.time_to_speed)
                    {
                      joint_S1.motion_steps_remaining += kbhit_J_S1_steps;
                      joint_S1.total_motion_time += calc_Steps2Time(abs(kbhit_J_S1_steps), JOINT_S1_TIMER_FAST_PERIOD);
                    }
                  }
                  else
                  {
                    if (joint_S1.discrete_time < joint_S1.total_motion_time - joint_S1.time_to_speed)
                    {
                      int time_to_eliminate = joint_S1.total_motion_time - joint_S1.time_to_speed - joint_S1.discrete_time;
                      int steps_to_eliminate = calc_Time2Steps(time_to_eliminate, JOINT_S1_TIMER_FAST_PERIOD);

                      joint_S1.motion_steps_remaining -= (joint_S1.motion_steps_remaining > 0) ? steps_to_eliminate : -steps_to_eliminate;
                      joint_S1.total_motion_time -= time_to_eliminate; 
                    }
                  }
                  break;
                }
                default:
                {
                  break;
                }
              }
            }

            if (   abs(kbhit_J_S2_steps) > 0
                && (interactive_stepper_safety == INTERACTIVE_STEPPER_SAFETY_LOW || (interactive_stepper_safety == INTERACTIVE_STEPPER_SAFETY_HIGH && abs(kbhit_J_S2_steps) <= interactive_joint_step_limit)))

            {
              switch (joint_S2.state)
              {
                case JOINT_STATE_STEPPER_HOLDING:
                {
                  joint_S2.motion_steps_remaining += kbhit_J_S2_steps;
                  joint_S2.current_period = JOINT_S2_TIMER_STOP_PERIOD;
                  joint_S2.state = JOINT_STATE_STEPPER_MOVING_FORWARD;

                  joint_S2.steps_to_speed = calc_SmoothStepsToSpeed(abs(joint_S2.motion_steps_remaining), JOINT_S2_FAST_SPEED, JOINT_S2_STEPS_TO_FAST_SPEED, joint_S2.final_speed);                  
                  joint_S2.total_motion_time = calc_SmoothMotionTimes(joint_S2.final_speed, abs(joint_S2.motion_steps_remaining), joint_S2.steps_to_speed, joint_S2.time_to_speed);
                  common_total_motion_time = max(joint_S2.total_motion_time, common_total_motion_time);

                  joint_S2.discrete_time = 0;
                  joint_S2.assumed_time = 0.0;
                  joint_S2.total_steps_made = 0;

                  ++active_joints_count;
                  main_state = MAIN_STATE_STEPPER_ACTIVE;
                  break;
                }
                case JOINT_STATE_STEPPER_MOVING_FORWARD:
                {
                  if ((joint_S2.motion_steps_remaining > 0 && kbhit_J_S2_steps > 0) || (joint_S2.motion_steps_remaining < 0 && kbhit_J_S2_steps < 0))
                  {
                    if (joint_S2.discrete_time < joint_S2.total_motion_time - joint_S2.time_to_speed)
                    {
                      joint_S2.motion_steps_remaining += kbhit_J_S2_steps;
                      joint_S2.total_motion_time += calc_Steps2Time(abs(kbhit_J_S2_steps), JOINT_S2_TIMER_FAST_PERIOD);
                    }
                  }
                  else
                  {
                    if (joint_S2.discrete_time < joint_S2.total_motion_time - joint_S2.time_to_speed)
                    {
                      int time_to_eliminate = joint_S2.total_motion_time - joint_S2.time_to_speed - joint_S2.discrete_time;
                      int steps_to_eliminate = calc_Time2Steps(time_to_eliminate, JOINT_S2_TIMER_FAST_PERIOD);

                      joint_S2.motion_steps_remaining -= (joint_S2.motion_steps_remaining > 0) ? steps_to_eliminate : -steps_to_eliminate;
                      joint_S2.total_motion_time -= time_to_eliminate; 
                    }
                  }
                  break;
                }
                default:
                {
                  break;
                }
              }
            }
            
            if (   abs(kbhit_J_E1_steps) > 0
                && (interactive_stepper_safety == INTERACTIVE_STEPPER_SAFETY_LOW || (interactive_stepper_safety == INTERACTIVE_STEPPER_SAFETY_HIGH && abs(kbhit_J_E1_steps) <= interactive_joint_step_limit)))
            {
              switch (joint_E1.state)
              {
                case JOINT_STATE_STEPPER_HOLDING:
                {
                  joint_E1.motion_steps_remaining += kbhit_J_E1_steps;
                  joint_E1.current_period = JOINT_E1_TIMER_STOP_PERIOD;
                  joint_E1.state = JOINT_STATE_STEPPER_MOVING_FORWARD;

                  joint_E1.steps_to_speed = calc_SmoothStepsToSpeed(abs(joint_E1.motion_steps_remaining), JOINT_E1_FAST_SPEED, JOINT_E1_STEPS_TO_FAST_SPEED, joint_E1.final_speed);
                  joint_E1.total_motion_time = calc_SmoothMotionTimes(joint_E1.final_speed, abs(joint_E1.motion_steps_remaining), joint_E1.steps_to_speed, joint_E1.time_to_speed);                  
                  common_total_motion_time = max(joint_E1.total_motion_time, common_total_motion_time);

                  joint_E1.discrete_time = 0;
                  joint_E1.assumed_time = 0.0;
                  joint_E1.total_steps_made = 0;

                  ++active_joints_count;
                  main_state = MAIN_STATE_STEPPER_ACTIVE;
                  break;
                }
                case JOINT_STATE_STEPPER_MOVING_FORWARD:
                {
                  if ((joint_E1.motion_steps_remaining > 0 && kbhit_J_E1_steps > 0) || (joint_E1.motion_steps_remaining < 0 && kbhit_J_E1_steps < 0))
                  {
                    if (joint_E1.discrete_time < joint_E1.total_motion_time - joint_E1.time_to_speed)
                    {
                      joint_E1.motion_steps_remaining += kbhit_J_E1_steps;
                      joint_E1.total_motion_time += calc_Steps2Time(abs(kbhit_J_E1_steps), JOINT_E1_TIMER_FAST_PERIOD);
                    }
                  }
                  else
                  {
                    if (joint_E1.discrete_time < joint_E1.total_motion_time - joint_E1.time_to_speed)
                    {
                      int time_to_eliminate = joint_E1.total_motion_time - joint_E1.time_to_speed - joint_E1.discrete_time;
                      int steps_to_eliminate = calc_Time2Steps(time_to_eliminate, JOINT_E1_TIMER_FAST_PERIOD);

                      joint_E1.motion_steps_remaining -= (joint_E1.motion_steps_remaining > 0) ? steps_to_eliminate : -steps_to_eliminate;
                      joint_E1.total_motion_time -= time_to_eliminate; 
                    }
                  }
                  break;
                }
                default:
                {
                  break;
                }
              }
            }

            if (    abs(kbhit_J_E2_steps) > 0
                && (interactive_stepper_safety == INTERACTIVE_STEPPER_SAFETY_LOW || (interactive_stepper_safety == INTERACTIVE_STEPPER_SAFETY_HIGH && abs(kbhit_J_E2_steps) <= interactive_joint_step_limit)))
            {
              switch (joint_E2.state)
              {
                case JOINT_STATE_STEPPER_HOLDING:
                {
                  joint_E2.motion_steps_remaining += kbhit_J_E2_steps;
                  joint_E2.current_period = JOINT_E2_TIMER_STOP_PERIOD;
                  joint_E2.state = JOINT_STATE_STEPPER_MOVING_FORWARD;

                  joint_E2.steps_to_speed = calc_SmoothStepsToSpeed(abs(joint_E2.motion_steps_remaining), JOINT_E2_FAST_SPEED, JOINT_E2_STEPS_TO_FAST_SPEED, joint_E2.final_speed);
                  joint_E2.total_motion_time = calc_SmoothMotionTimes(joint_E2.final_speed, abs(joint_E2.motion_steps_remaining), joint_E2.steps_to_speed, joint_E2.time_to_speed);
                  common_total_motion_time = max(joint_E2.total_motion_time, common_total_motion_time);

                  joint_E2.discrete_time = 0;
                  joint_E2.assumed_time = 0.0;
                  joint_E2.total_steps_made = 0;

                  ++active_joints_count;
                  main_state = MAIN_STATE_STEPPER_ACTIVE;
                  break;
                }
                case JOINT_STATE_STEPPER_MOVING_FORWARD:
                {
                  if ((joint_E2.motion_steps_remaining > 0 && kbhit_J_E2_steps > 0) || (joint_E2.motion_steps_remaining < 0 && kbhit_J_E2_steps < 0))
                  {
                    if (joint_E2.discrete_time < joint_E2.total_motion_time - joint_E2.time_to_speed)
                    {
                      joint_E2.motion_steps_remaining += kbhit_J_E2_steps;
                      joint_E2.total_motion_time += calc_Steps2Time(abs(kbhit_J_E2_steps), JOINT_E2_TIMER_FAST_PERIOD);
                    }
                  }
                  else
                  {
                    if (joint_E2.discrete_time < joint_E2.total_motion_time - joint_E2.time_to_speed)
                    {
                      int time_to_eliminate = joint_E2.total_motion_time - joint_E2.time_to_speed - joint_E2.discrete_time;
                      int steps_to_eliminate = calc_Time2Steps(time_to_eliminate, JOINT_E2_TIMER_FAST_PERIOD);

                      joint_E2.motion_steps_remaining -= (joint_E2.motion_steps_remaining > 0) ? steps_to_eliminate : -steps_to_eliminate;
                      joint_E2.total_motion_time -= time_to_eliminate; 
                    }
                  }
                  break;
                }
                default:
                {
                  break;
                }
              }
            }

            if (    abs(kbhit_J_W1_steps) > 0
                && (interactive_stepper_safety == INTERACTIVE_STEPPER_SAFETY_LOW || (interactive_stepper_safety == INTERACTIVE_STEPPER_SAFETY_HIGH && abs(kbhit_J_W1_steps) <= interactive_joint_step_limit)))
            {
              switch (joint_W1.state)
              {
                case JOINT_STATE_STEPPER_HOLDING:
                {
                  joint_W1.motion_steps_remaining += kbhit_J_W1_steps;
                  joint_W1.current_period = JOINT_W1_TIMER_STOP_PERIOD;
                  joint_W1.state = JOINT_STATE_STEPPER_MOVING_FORWARD;

                  joint_W1.steps_to_speed = calc_SmoothStepsToSpeed(abs(joint_W1.motion_steps_remaining), JOINT_W1_FAST_SPEED, JOINT_W1_STEPS_TO_FAST_SPEED, joint_W1.final_speed);                  
                  joint_W1.total_motion_time = calc_SmoothMotionTimes(joint_W1.final_speed, abs(joint_W1.motion_steps_remaining), joint_W1.steps_to_speed, joint_W1.time_to_speed);                  
                  common_total_motion_time = max(joint_W1.total_motion_time, common_total_motion_time);

                  joint_W1.discrete_time = 0;
                  joint_W1.assumed_time = 0.0;
                  joint_W1.total_steps_made = 0;

                  ++active_joints_count;
                  main_state = MAIN_STATE_STEPPER_ACTIVE;
                  break;
                }
                case JOINT_STATE_STEPPER_MOVING_FORWARD:
                {
                  if ((joint_W1.motion_steps_remaining > 0 && kbhit_J_W1_steps > 0) || (joint_W1.motion_steps_remaining < 0 && kbhit_J_W1_steps < 0))
                  {
                    if (joint_W1.discrete_time < joint_W1.total_motion_time - joint_W1.time_to_speed)
                    {
                      joint_W1.motion_steps_remaining += kbhit_J_W1_steps;
                      joint_W1.total_motion_time += calc_Steps2Time(abs(kbhit_J_W1_steps), JOINT_W1_TIMER_FAST_PERIOD);
                    }
                  }
                  else
                  {
                    if (joint_W1.discrete_time < joint_W1.total_motion_time - joint_W1.time_to_speed)
                    {
                      int time_to_eliminate = joint_W1.total_motion_time - joint_W1.time_to_speed - joint_W1.discrete_time;
                      int steps_to_eliminate = calc_Time2Steps(time_to_eliminate, JOINT_W1_TIMER_FAST_PERIOD);

                      joint_W1.motion_steps_remaining -= (joint_W1.motion_steps_remaining > 0) ? steps_to_eliminate : -steps_to_eliminate;
                      joint_W1.total_motion_time -= time_to_eliminate; 
                    }
                  }                  
                  break;
                }
                default:
                {
                  break;
                }
              }
            }
            
            if (    abs(kbhit_J_W2_steps) > 0
                && (interactive_stepper_safety == INTERACTIVE_STEPPER_SAFETY_LOW || (interactive_stepper_safety == INTERACTIVE_STEPPER_SAFETY_HIGH && abs(kbhit_J_W2_steps) <= interactive_joint_step_limit)))
            {
              switch (joint_W2.state)
              {
                case JOINT_STATE_STEPPER_HOLDING:
                {
                  joint_W2.motion_steps_remaining += kbhit_J_W2_steps;
                  joint_W2.current_period = JOINT_W2_TIMER_STOP_PERIOD;
                  joint_W2.state = JOINT_STATE_STEPPER_MOVING_FORWARD;

                  joint_W2.steps_to_speed = calc_SmoothStepsToSpeed(abs(joint_W2.motion_steps_remaining), JOINT_W2_FAST_SPEED, JOINT_W2_STEPS_TO_FAST_SPEED, joint_W2.final_speed);
                  joint_W2.total_motion_time = calc_SmoothMotionTimes(joint_W2.final_speed, abs(joint_W2.motion_steps_remaining), joint_W2.steps_to_speed, joint_W2.time_to_speed);                  
                  common_total_motion_time = max(joint_W2.total_motion_time, common_total_motion_time);

                  joint_W2.discrete_time = 0;
                  joint_W2.assumed_time = 0.0;
                  joint_W2.total_steps_made = 0;

                  ++active_joints_count;
                  main_state = MAIN_STATE_STEPPER_ACTIVE;
                  break;
                }
                case JOINT_STATE_STEPPER_MOVING_FORWARD:
                {
                  if ((joint_W2.motion_steps_remaining > 0 && kbhit_J_W2_steps > 0) || (joint_W2.motion_steps_remaining < 0 && kbhit_J_W2_steps < 0))
                  {
                    if (joint_W2.discrete_time < joint_W2.total_motion_time - joint_W2.time_to_speed)
                    {
                      joint_W2.motion_steps_remaining += kbhit_J_W2_steps;
                      joint_W2.total_motion_time += calc_Steps2Time(abs(kbhit_J_W2_steps), JOINT_W2_TIMER_FAST_PERIOD);
                    }
                  }
                  else
                  {
                    if (joint_W2.discrete_time < joint_W2.total_motion_time - joint_W2.time_to_speed)
                    {
                      int time_to_eliminate = joint_W2.total_motion_time - joint_W2.time_to_speed - joint_W2.discrete_time;
                      int steps_to_eliminate = calc_Time2Steps(time_to_eliminate, JOINT_W2_TIMER_FAST_PERIOD);

                      joint_W2.motion_steps_remaining -= (joint_W2.motion_steps_remaining > 0) ? steps_to_eliminate : -steps_to_eliminate;
                      joint_W2.total_motion_time -= time_to_eliminate; 
                    }
                  }                                    
                  break;
                }
                default:
                {
                  break;
                }
              }
            }

            if (   abs(kbhit_J_G_steps) > 0
                && (interactive_stepper_safety == INTERACTIVE_STEPPER_SAFETY_LOW || (interactive_stepper_safety == INTERACTIVE_STEPPER_SAFETY_HIGH && abs(kbhit_J_G_steps) <= interactive_joint_step_limit)))
            {
              switch (joint_G.state)
              {
                case JOINT_STATE_STEPPER_HOLDING:
                {
                  joint_G.motion_steps_remaining += kbhit_J_G_steps;
                  joint_G.current_period = JOINT_G_TIMER_STOP_PERIOD;
                  joint_G.state = JOINT_STATE_STEPPER_MOVING_FORWARD;

                  joint_G.steps_to_speed = calc_SmoothStepsToSpeed(abs(joint_G.motion_steps_remaining), JOINT_G_FAST_SPEED, JOINT_G_STEPS_TO_FAST_SPEED, joint_G.final_speed);
                  joint_G.total_motion_time = calc_SmoothMotionTimes(joint_G.final_speed, abs(joint_G.motion_steps_remaining), joint_G.steps_to_speed, joint_G.time_to_speed);
                  common_total_motion_time = max(joint_G.total_motion_time, common_total_motion_time);

                  joint_G.discrete_time = 0;
                  joint_G.assumed_time = 0.0;
                  joint_G.total_steps_made = 0;

                  ++active_joints_count;
                  main_state = MAIN_STATE_STEPPER_ACTIVE;
                  break;
                }
                case JOINT_STATE_STEPPER_MOVING_FORWARD:
                {
                  if ((joint_G.motion_steps_remaining > 0 && kbhit_J_G_steps > 0) || (joint_G.motion_steps_remaining < 0 && kbhit_J_G_steps < 0))
                  {
                    if (joint_G.discrete_time < joint_G.total_motion_time - joint_G.time_to_speed)
                    {
                      joint_G.motion_steps_remaining += kbhit_J_G_steps;
                      joint_G.total_motion_time += calc_Steps2Time(abs(kbhit_J_G_steps), JOINT_G_TIMER_FAST_PERIOD);
                    }
                  }
                  else
                  {
                    if (joint_G.discrete_time < joint_G.total_motion_time - joint_G.time_to_speed)
                    {
                      int time_to_eliminate = joint_G.total_motion_time - joint_G.time_to_speed - joint_G.discrete_time;
                      int steps_to_eliminate = calc_Time2Steps(time_to_eliminate, JOINT_G_TIMER_FAST_PERIOD);

                      joint_G.motion_steps_remaining -= (joint_G.motion_steps_remaining > 0) ? steps_to_eliminate : -steps_to_eliminate;
                      joint_G.total_motion_time -= time_to_eliminate; 
                    }
                  }                           
                  break;
                }
                default:
                {
                  break;
                }
              }
            }
                        
/*
              Serial.print("J_S1: ");
              Serial.print(kbhit_J_S1_steps);
              Serial.println();

              Serial.print("J_S2: ");
              Serial.print(kbhit_J_S2_steps);
              Serial.println();

              Serial.print("J_E1: ");
              Serial.print(kbhit_J_E1_steps);
              Serial.println();

              Serial.print("J_E2: ");
              Serial.print(kbhit_J_E2_steps);
              Serial.println();

              Serial.print("J_W1: ");
              Serial.print(kbhit_J_W1_steps);
              Serial.println();

              Serial.print("J_W2: ");
              Serial.print(kbhit_J_W2_steps);
              Serial.println();

              Serial.print("J_G:  ");
              Serial.print(kbhit_J_G_steps);
              Serial.println();
            */

              if (common_total_motion_time > 0)
              {
                if (abs(kbhit_J_S1_steps) > 0)
                {
                  joint_S1.equalize_MotionTime(common_total_motion_time);
                }
                if (abs(kbhit_J_S2_steps) > 0)
                {
                  joint_S2.equalize_MotionTime(common_total_motion_time);
                }
                if (abs(kbhit_J_E1_steps) > 0)
                {
                  joint_E1.equalize_MotionTime(common_total_motion_time);
                }
                if (abs(kbhit_J_E2_steps) > 0)
                {
                  joint_E2.equalize_MotionTime(common_total_motion_time);
                }
                if (abs(kbhit_J_W1_steps) > 0)
                {
                  joint_W1.equalize_MotionTime(common_total_motion_time);
                }
                if (abs(kbhit_J_W2_steps) > 0)
                {
                  joint_W2.equalize_MotionTime(common_total_motion_time);
                }
                if (abs(kbhit_J_G_steps) > 0)
                {
                  joint_G.equalize_MotionTime(common_total_motion_time);
                }
              }
          }
        }
        input_rd = 0;
      }
    }

    serial_communication_timeout = SERIAL_COMMUNICATION_PERIOD;
  }
}
