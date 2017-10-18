#include "SearchController.h"
#include <angles/angles.h>

using namespace std;

SearchController::SearchController() {
  rng = new random_numbers::RandomNumberGenerator();
  currentLocation.x = 0;
  currentLocation.y = 0;
  currentLocation.theta = 0;

  centerLocation.x = 0;
  centerLocation.y = 0;
  centerLocation.theta = 0;
  result.PIDMode = FAST_PID;

  result.fingerAngle = M_PI/2;
  result.wristAngle = M_PI/4;
}

void SearchController::Reset() {
  result.reset = false;
  attemptCount = 0;
}

/**
 * This code implements a basic random walk search.
 */
Result SearchController::DoWork() {

  if (giveUpSearching()) 
  {
    informed_search = false;
    result.type = behavior;
    result.b = COMPLETED;
    result.reset = true;
    return result;
  }

  if (!result.wpts.waypoints.empty()) {
    if (hypot(result.wpts.waypoints[0].x-currentLocation.x, result.wpts.waypoints[0].y-currentLocation.y) < 0.15) {
      attemptCount = 0;
    }
  }

  if (attemptCount > 0 && attemptCount < 5) {
    attemptCount++;
    return result;
  }
  else if (attemptCount >= 5 || attemptCount == 0) 
  {
    attemptCount = 1;


    result.type = waypoint;
    Point  searchLocation;

    float correlation = CPFA_parameters.uninformed_search_variation;

    if (informed_search)
    {
      cout << "Informed Search" << endl;
      float exponential = exp(-CPFA_parameters.rate_of_informed_search_decay * informed_search_time);
      exponential *= (4 * M_PI - CPFA_parameters.uninformed_search_variation);
      correlation += exponential;
    }
    else 
    {
      cout << "Uninformed Search" << endl;
    }

    searchLocation.theta = rng->gaussian(currentLocation.theta, correlation); 
    searchLocation.x = currentLocation.x + search_step_size*cos(searchLocation.theta);
    searchLocation.y = currentLocation.y + search_step_size*sin(searchLocation.theta);

    result.wpts.waypoints.clear();
    result.wpts.waypoints.insert(result.wpts.waypoints.begin(), searchLocation);
    
    return result;
  }

}

void SearchController::SetCenterLocation(Point centerLocation) {
  
  float diffX = this->centerLocation.x - centerLocation.x;
  float diffY = this->centerLocation.y - centerLocation.y;
  this->centerLocation = centerLocation;
  
  if (!result.wpts.waypoints.empty())
  {
  result.wpts.waypoints.back().x -= diffX;
  result.wpts.waypoints.back().y -= diffY;
  }
  
}

void SearchController::SetCurrentLocation(Point currentLocation) {
  this->currentLocation = currentLocation;
}

void SearchController::ProcessData() {
}

bool SearchController::ShouldInterrupt(){
  ProcessData();

  return false;
}

bool SearchController::HasWork() {
  return true;
}

void SearchController::SetSuccesfullPickup() {
  Reset();
}

void SearchController::setSearchType(bool informed_search) 
{
  this->informed_search = informed_search;

  if(informed_search)
  {
    informed_search_time = current_time;
  }
}

void SearchController::SetCurrentTimeInMilliSecs( long int time )
{
  current_time = time;
}

bool SearchController::giveUpSearching() 
{
  double random_num = rng->uniformReal(0, 1);
  bool give_up = false;

  if (CPFA_parameters.probability_of_returning_to_nest > random_num)  
  {
    give_up = true;
  } 

  return give_up;
}
