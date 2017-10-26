#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <algorithm>
#include <thread>
#include <vector>

using namespace std;

// Constants
int max_lanes = 3;

double getDistanceWithCarCost(double distance)
{
  double cost = 0.0;
  if (distance > 70)
  {
    cost = 0.0;
  }
  else if (distance < 30 && distance > 0)
  {
    cost = 1.0;
  }
  else if (distance <= 70 && distance >= 30)
  {
    cost = 1.0 - ((distance - 30) / 40);
  }
  return cost;
}

double stayOnTheRoadCost(int lane) {
  if (lane < 0 or lane >= max_lanes)
  {
    return 1.0;
  }
  else
  {
    return 0.0;
  }
}

double changeLaneCost()
{
    return 1.0;
}

double getSpeedCost(double velocity, bool is_accelerating) {

  double cost = 0.0;
  double speed_limit = 50.0;
  double buffer_v = 3.0;
  double target_speed = speed_limit - buffer_v;
  double stop_cost = 1.0;

  if (is_accelerating) {
    cost = 0.0;
  }
  else
  {
    if (velocity < target_speed)
    {
      cost = stop_cost * ((target_speed - velocity) / target_speed);
    }
    else if (velocity > speed_limit)
    {
      cost = 1.0;
    }
    else if ((velocity > target_speed) && (velocity < speed_limit))
    {
      cost = 0.0;
    }
  }

  return cost;
}

double avoidCollisionCost(double distance)
{
  if (distance > 10)
  {
    return 0.0;
  }
  else
  {
    return 1.0;
  }
}
