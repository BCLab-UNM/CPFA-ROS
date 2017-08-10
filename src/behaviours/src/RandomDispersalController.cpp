#include "RandomDispersalController.h"

RandomDispersalController::RandomDispersalController()
{
  srand(CPFA_parameters.random_seed);
}

RandomDispersalController::~RandomDispersalController() {}

void RandomDispersalController::Reset() {}

Result RandomDispersalController::DoWork()
{
  Result result;

  if (switch_to_search)
  {
    //change from traveling state to uninformed search state
    goal_location_set = false;
    switch_to_search = false;
    result.type = behavior;
    result.b = COMPLETED;
  }
  //first run of Random Dsipersal Controller 
  else if (!init)
  {
      goal_location_set = true;
      init = true;
      //pick random heading that is in the 180 degree arc away from the center
      goal_location.theta = current_location.theta + rand() * (M_PI)/INT_MAX + M_PI/2;

  }
  else
  {
    // Initial random heading not set
    if (goal_location_set)
    {
      result.type = precisionDriving;
      result.PIDMode = FAST_PID;
      result.pd.cmdVel = travel_speed;

      float angular_error = 0;
      //calculate anguar error between current heading and desired heading
      angular_error = angles::shortest_angular_distance(current_location.theta, goal_location.theta);

      result.pd.cmdAngularError = angular_error;


    }
    else
    {
      goal_location_set = true;
      //select random heading in any direction
      goal_location.theta = rand() * (2.0 * M_PI)/INT_MAX;
    }
  }

  return result;
}

void RandomDispersalController::ProcessData()
{
  if (rand() * 1.0f/INT_MAX < CPFA_parameters.probability_of_switching_to_searching)
  {
    //switch to uninformed coralated search
    switch_to_search = true;
  }
}

bool RandomDispersalController::ShouldInterrupt()
{
  ProcessData();

  bool interrupt = false;

  if (switch_to_search)
  {
    interrupt = true;
  }


  return interrupt;
}


bool RandomDispersalController::HasWork()
{
  return true;
}
