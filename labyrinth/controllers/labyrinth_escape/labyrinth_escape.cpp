#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/DistanceSensor.hpp>
#include <webots/Gyro.hpp>

#define TIME_STEP 64

constexpr int DS_COUNT = 3;
constexpr int FRONT = 0;
constexpr int FRONT_RIGHT = 1;
constexpr int RIGHT = 2;
constexpr float MAX_SPEED = 6.00;

// All the webots classes are defined in the "webots" namespace
using namespace webots;

int main(int argc, char **argv) {
  Robot *robot = new Robot();

  // get the motor devices
  Motor *leftMotor = robot->getMotor("left wheel motor");
  Motor *rightMotor = robot->getMotor("right wheel motor");
  // set the target position of the motors
  leftMotor->setPosition(INFINITY);
  rightMotor->setPosition(INFINITY);
 
  leftMotor->setVelocity(0.0);
  rightMotor->setVelocity(0.0);
 
  DistanceSensor* ds[DS_COUNT];
  for (int i = 0; i < DS_COUNT; i++)
  {
    std::string ds_name = "ps" + std::to_string(i);
    ds[i] = robot->getDistanceSensor(ds_name);
    ds[i]->enable(TIME_STEP);
  }
  
  float ds_values[DS_COUNT];
  
  float left_speed = 0.0;
  float right_speed = 0.0;

  while (robot->step(TIME_STEP) != -1) {
    for (int i = 0; i < DS_COUNT; i++)
    {
      ds_values[i] = ds[i]->getValue();
    }

    bool front_obstacle = ds_values[FRONT] > 75.0;
    bool right_obstacle = ds_values[RIGHT] > 75.0;
    bool corner = ds_values[FRONT_RIGHT] > 75.0;

    if (front_obstacle == true)
    {
      left_speed = -MAX_SPEED;
      right_speed = MAX_SPEED;
    }
    else
    {
        if (right_obstacle == true)
        {
          left_speed = MAX_SPEED;
          right_speed = MAX_SPEED;
        }
        else
        {
          left_speed = MAX_SPEED;
          right_speed = MAX_SPEED/8;
        }
        
        if (corner == true)
        {
          left_speed = MAX_SPEED/8;
          right_speed = MAX_SPEED;
        }
    }
    
    leftMotor->setVelocity(left_speed);
    rightMotor->setVelocity(right_speed);
  }

 delete robot;

 return 0;
}