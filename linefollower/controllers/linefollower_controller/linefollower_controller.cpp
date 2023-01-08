#include <webots/Robot.hpp>
#include <webots/DistanceSensor.hpp>
#include <webots/Motor.hpp>
#include <iostream>
// #include <cmath>

constexpr float TIME_STEP = 32;
constexpr float MAX_SPEED = 6.28;
constexpr int DS_COUNT = 3;
constexpr int RIGHT = 0;
constexpr int MIDDLE = 1;
constexpr int LEFT = 2;
constexpr int WHEEL_COUNT = 4;

// All the webots classes are defined in the "webots" namespace
using namespace webots;

// inline bool float_comparison(double val1, double val2)
// {
  // bool retVal = val1 - val2 < 0 ? true : false;
  // std::cout<<retVal<<" comp " <<val1 - val2<<"\n";
  // return retVal;
// }

// This is the main program of your controller.
int main(int argc, char **argv) {
  // create the Robot instance.
  Robot *robot = new Robot();

  // get the time step of the current world.
  // int timeStep = (int)robot->getBasicTimeStep();
  
  Motor* wheels[WHEEL_COUNT];
  char wheels_names[WHEEL_COUNT][9] = { "wheel_r1", "wheel_l1", "wheel_r2", "wheel_l2" };
  for (int i = 0; i < WHEEL_COUNT; i++)
  {
      wheels[i] = robot->getMotor(wheels_names[i]);
      wheels[i]->setPosition(INFINITY);
      wheels[i]->setVelocity(0.0);
  }
  
  DistanceSensor* ds[DS_COUNT];
  char dsNames[DS_COUNT][10] = {"ds_right", "ds_middle", "ds_left"};
  for (int i = 0; i < DS_COUNT; i++)
  {
      ds[i] = robot->getDistanceSensor(dsNames[i]);
      ds[i]->enable(TIME_STEP);
  }
  
  float dsValues[DS_COUNT];
  
  float leftSpeed = 0.0;
  float rightSpeed = 0.0;
  
  // Main loop:
  while (robot->step(TIME_STEP) != -1) {
    
    for (int i = 0; i < DS_COUNT; i++)
      {
        dsValues[i] = ds[i]->getValue();
      }
    
    bool noTrack = dsValues[RIGHT] > 715.0 && dsValues[RIGHT] < 720.0 &&
                   dsValues[MIDDLE] > 715.0 && dsValues[MIDDLE] < 720.0 &&
                   dsValues[LEFT] > 715.0 && dsValues[LEFT] < 720.0;
    bool allOnTrack = dsValues[RIGHT] > 695.0 && dsValues[RIGHT] < 705.0 &&
                   dsValues[MIDDLE] > 695.0 && dsValues[MIDDLE] < 705.0 &&
                   dsValues[LEFT] > 695.0 && dsValues[LEFT] < 705.0;
    
    do {
      if (noTrack || allOnTrack)
      {
        leftSpeed = 0.4 * MAX_SPEED;
        rightSpeed = 0.4 * MAX_SPEED;
        std::cout<<"Go straight\n";
        continue;
      }
      else
      {
        if (static_cast<int>(dsValues[RIGHT]) > static_cast<int>(dsValues[LEFT]))
        {
          rightSpeed = 0.5 * MAX_SPEED;
          leftSpeed = 0.2 * MAX_SPEED;
          std::cout<<"Turn right\n";
        }
        else
        {
          rightSpeed = 0.2 * MAX_SPEED;
          leftSpeed = 0.5 * MAX_SPEED;
          std::cout<<"Turn left\n";
        }
        
      }
    } while (0);
  
    wheels[0]->setVelocity(rightSpeed);
    wheels[1]->setVelocity(leftSpeed);
    wheels[2]->setVelocity(rightSpeed);
    wheels[3]->setVelocity(leftSpeed);
  };

  // Enter here exit cleanup code.

  delete robot;
  return 0;
}
