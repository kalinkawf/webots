#include <webots/Robot.hpp>
#include <webots/DistanceSensor.hpp>
#include <webots/Motor.hpp>
#include <iostream>

// All the webots classes are defined in the "webots" namespace
using namespace webots;

constexpr float TIME_STEP = 64.0;
constexpr float MAX_FORWARD = 6.28;
constexpr float MAX_BACKWARD = -6.28;

constexpr int DS_COUNT = 2;
constexpr int EDGE_SENSOR = 0;
constexpr int OBSTACLE_SENSOR = 1;

// state machine
enum robot_states{
  LOOK_FOR = 0, // looks for object
  FOUND,        // object is found in front
  JUST_FOUND,   // object is found, but additonal rotaton is needed
  LOST,         // object is lost
};

static float timer = 0.0;

// updates timer up or down
inline void timerUpdate(float num)
{
  timer += (num / TIME_STEP);
  if (timer < 0.0) { timer = 0.0; }
}

inline void speedUpdate(float &speed, float value)
{
  speed = value;
}

// This is the main program of your controller.
// It creates an instance of your Robot instance, launches its
// function(s) and destroys it at the end of the execution.
// Note that only one instance of Robot should be created in
// a controller program.
// The arguments of the main function can be specified by the
// "controllerArgs" field of the Robot node
int main(int argc, char **argv) {
  // create the Robot instance.
  Robot *robot = new Robot();
  
  // get the time step of the current world.
  //int timeStep = (int)robot->getBasicTimeStep();
  
  DistanceSensor* ds[DS_COUNT];
  char dsNames[DS_COUNT][12] = {"ds_edge", "ds_obstacle"};
  for (int i = 0; i < DS_COUNT; i++) {
        ds[i] = robot->getDistanceSensor(dsNames[i]);
        ds[i]->enable(TIME_STEP);
  }
  
  // create wheels instance
  Motor* wheels[4];
  char wheels_names[4][9] = { "wheel_r1", "wheel_l1", "wheel_r2", "wheel_l2"};
  for (int i = 0; i < 4; i++) {
    wheels[i] = robot->getMotor(wheels_names[i]);
    wheels[i]->setPosition(INFINITY);
    wheels[i]->setVelocity(0.0);
  }
  
  float leftSpeed = 0.0;
  float rightSpeed = 0.0;
  
  static robot_states state = LOOK_FOR;
  static bool rotate = true;
  
  while (robot->step(TIME_STEP) != -1) {
  
    double dsValues[DS_COUNT];
    for (int i = 0; i < DS_COUNT; i++)
      {
          dsValues[i] = ds[i]->getValue();
      }
    
    bool onFloor = dsValues[EDGE_SENSOR] < 1000;
    bool obstacleInRange = dsValues[OBSTACLE_SENSOR] < 1000;
    
    std::cout<<timer<<"     "<<state<<"     "<<rotate<<"\n";
    
    if (state == LOOK_FOR)
    {
      if (obstacleInRange && timer > 0)
      {
        rotate = true;
        state = JUST_FOUND;
        timer = 1.0;
      }
      else if (obstacleInRange && timer == 0)
      {
        state = FOUND;
      }
      else
      {
        timerUpdate(1.0);
        rightSpeed = MAX_FORWARD;
        leftSpeed = MAX_BACKWARD;
      }
    }
    else if (state == JUST_FOUND)
    {
      if (timer > 0 && obstacleInRange && rotate)
      {
        timerUpdate(-1.0);
        rightSpeed = MAX_FORWARD;
        leftSpeed = MAX_BACKWARD;
      }
      else
      {
        rotate = false;
        if (onFloor)
        {
          if (obstacleInRange)
          {
            timerUpdate(1.0);
            rightSpeed = 0.2 * MAX_FORWARD;
            leftSpeed = 0.2 * MAX_FORWARD;
          }
          else
          {
            state = LOST;
            timerUpdate(-1.0);
          }
        }
        else
        {
          state = LOST;
          timerUpdate(-1.0);
          rightSpeed = 0.2 * MAX_BACKWARD;
          leftSpeed = 0.2 * MAX_BACKWARD;
        }
      }
    }
    else if (state == FOUND)
    {
      if (onFloor)
      {
        if (obstacleInRange)
        {
          timerUpdate(1.0);
          rightSpeed = 0.2 * MAX_FORWARD;
          leftSpeed = 0.2 * MAX_FORWARD;
        }
        else
        {
          state = LOST;
          timerUpdate(-1.0);
        }
      }
      else
      {
        state = LOST;
        timerUpdate(-1.0);
        rightSpeed = 0.2 * MAX_BACKWARD;
        leftSpeed = 0.2 * MAX_BACKWARD;
      }
    }
    else // state lost
    {
      if (timer > 0)
      {
        timerUpdate(-1.0);
        rightSpeed = 0.2 * MAX_BACKWARD;
        leftSpeed = 0.2 * MAX_BACKWARD;
      }
      else
      {
        state = LOOK_FOR;
        rightSpeed = 0.0;
        leftSpeed = 0.0;
      }
    }
    
    wheels[0]->setVelocity(rightSpeed);
    wheels[1]->setVelocity(leftSpeed);
    wheels[2]->setVelocity(rightSpeed);
    wheels[3]->setVelocity(leftSpeed);
  };

  // Enter here exit cleanup code.

  delete robot;
  return 0;
}
