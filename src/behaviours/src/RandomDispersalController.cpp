#include "RandomDispersalController.h"


using namespace std;
RandomDispersalController::RandomDispersalController()
{

}

RandomDispersalController::~RandomDispersalController() {}

void RandomDispersalController::Reset() {}
	
	
Result RandomDispersalController::DoWork()
{
	cout<<"RandomDispersalController::DoWork()"<<endl;
  Result result;
  has_control = true;

  result.type = precisionDriving;
  result.PIDMode = FAST_PID;
  
  cout << "dispersal doing work" << endl;

  if (switch_to_search)
  {
    //change from traveling state to uninformed search state
    goal_location_set = false;
    switch_to_search = false;
    has_control = false;
    result.type = behavior;
    result.b = COMPLETED;
  }
  //first run of Random Dsipersal Controller
  else if (!init)
  {
    if (CPFA_parameters.random_seed != -1)
    {
      srand(CPFA_parameters.random_seed);
    }
    else
    {
      srand(current_time/1e3);
    }
    goal_location_set = true;
    init = true;
    //pick random heading that is in the 180 degree arc away from the center
    goal_location.theta = current_location.theta + (rand() * (M_PI)/INT_MAX) + M_PI/2;

    cout << "first goal is : " << goal_location.theta << endl;

  }
  else
  {
    // Initial random heading not set
    if (goal_location_set)
    {

      float angular_error = 0;
      //calculate anguar error between current heading and desired heading
      angular_error = angles::shortest_angular_distance(current_location.theta, goal_location.theta);

      result.pd.cmdAngularError = angular_error;
      
      if (angular_error < 0.4)
      {
        result.pd.cmdVel = travel_speed;
      }
      else
      {
        result.pd.cmdVel = 0.0;
      }


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
  if (!switch_to_search && has_control)
  {
    if (rand() * 1.0f/INT_MAX < CPFA_parameters.probability_of_switching_to_searching)
    {
      //switch to uninformed coralated search
      switch_to_search = true;
    }
  }
}

CPFAState RandomDispersalController::GetCPFAState() 
{
  return cpfa_state;
}

void RandomDispersalController::SetCPFAState(CPFAState state) {
  cpfa_state = state;
  result.cpfa_state = state;
}


bool RandomDispersalController::ShouldInterrupt()
{
  ProcessData();

  bool interrupt = false;

  if (switch_to_search)
  {
    interrupt = true;
    has_control = false;
  }


  return interrupt;
}


bool RandomDispersalController::HasWork()
{
  cout << "RDC has work" << endl;
  return true;
}
