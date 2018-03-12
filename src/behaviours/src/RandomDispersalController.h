#ifndef RANDOM_DISPERSAL_CONTROLLER_H
#define RANDOM_DISPERSAL_CONTROLLER_H

#include "Controller.h"
#include "Point.h"
#include "Tag.h"
#include "Result.h"
#include "CPFAParameters.h"
#include <stdlib.h>
#include <angles/angles.h>

#include <math.h> // hypot
#include <climits>

class RandomDispersalController : virtual Controller {

public:
  RandomDispersalController();
  ~RandomDispersalController();
  
  void Reset() override;
  Result DoWork() override;
  bool ShouldInterrupt() override;
  bool HasWork() override;

  void SetCurrentLocation(Point current_location) {this->current_location = current_location;}
  void SetCurrentTime(long int time) {current_time = time;}
  void SetCPFAState(CPFAState state) override;
  CPFAState GetCPFAState() override;
private:

  CPFAState cpfa_state = start_state;
  //struct for returning data to the ROS adapter
  Result result;
  
  void ProcessData();
  
  Point current_location;
  Point goal_location;

  bool goal_location_set = false;
  bool init = false;
  bool switch_to_search = false;
  bool has_control = false;
  float travel_speed = 0.2;
  long int current_time;

  CPFAParameters CPFA_parameters;
};

#endif
