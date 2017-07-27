#include "CPFASearchController.h"

CPFASearchController::CPFASearchController() {
  rng = new random_numbers::RandomNumberGenerator();
  currentLocation.x = 0;
  currentLocation.y = 0;
  currentLocation.theta = 0;

  centerLocation.x = 0;
  centerLocation.y = 0;
  centerLocation.theta = 0;
  result.PIDMode = FAST_PID;
}

void CPFASearchController::Reset() {
  result.reset = false;
}

/**
 * This code implements a basic random walk search.
 */
Result CPFASearchController::DoWork() {

  if (!result.wpts.waypoints.empty()) {
    if (hypot(result.wpts.waypoints[0].x-currentLocation.x, result.wpts.waypoints[0].y-currentLocation.y) < 0.10) {
      attemptCount = 0;
    }
  }

  if (attemptCount > 0 && attemptCount < 5) {
    attemptCount++;
    if (succesfullPickup) {
      succesfullPickup = false;
      attemptCount = 1;
    }
    return result;
  }
  else if (attemptCount >= 5 || attemptCount == 0) {
    attemptCount = 1;


    result.type = waypoint;
    Point  searchLocation;

    //select new position 50 cm from current location
    if (first_waypoint)
    {
      first_waypoint = false;
      searchLocation.theta = currentLocation.theta + M_PI;
      searchLocation.x = currentLocation.x + (0.5 * cos(searchLocation.theta));
      searchLocation.y = currentLocation.y + (0.5 * sin(searchLocation.theta));
    }
    else
    {
      //select new heading from Gaussian distribution around current heading
      searchLocation.theta = rng->gaussian(currentLocation.theta, 0.25);
      searchLocation.x = currentLocation.x + (0.5 * cos(searchLocation.theta));
      searchLocation.y = currentLocation.y + (0.5 * sin(searchLocation.theta));
    }

    result.wpts.waypoints.clear();
    result.wpts.waypoints.insert(result.wpts.waypoints.begin(), searchLocation);

    return result;
  }

}

void CPFASearchController::SetCenterLocation(Point centerLocation) {
  this->centerLocation = centerLocation;
}

void CPFASearchController::SetCurrentLocation(Point currentLocation) {
  this->currentLocation = currentLocation;
}

void CPFASearchController::ProcessData() {
}

bool CPFASearchController::ShouldInterrupt(){
  ProcessData();

  return false;
}

bool CPFASearchController::HasWork() {
  return true;
}

void CPFASearchController::SetSuccesfullPickup() {
  succesfullPickup = true;
}


