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
  if (distance > 100)
  {
    cost = 0.0;
  }
  else if (distance < 30 && distance > 0)
  {
    cost = 1.0;
  }
  else if (distance <= 100 && distance >= 30)
  {
    cost = 1.0 - ((distance - 30) / 70);
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

double getSpeedCost(double velocity) {

  double cost = 0.0;
  double speed_limit = 50.0;
  double buffer_v = 2.0;
  double target_speed = speed_limit - buffer_v;
  double stop_cost = 1.0;
  double min_speed = 20.0;

  if (velocity < min_speed)
  {
    cost = 0.2;
  }
  else if (velocity >= min_speed and velocity < target_speed)
  {
    cost = stop_cost * ((target_speed - velocity) / target_speed);
  }
  else if (velocity > speed_limit)
  {
    cost = 1.0;
  }
  else if ((velocity > target_speed) && (velocity < speed_limit))
  {
    cost = (velocity - target_speed) / buffer_v;
  }

  return cost;
}

double avoidCollisionCost(double distance)
{
  if (distance > 20)
  {
    return 0.0;
  }
  else
  {
    return 1.0;
  }
}