#ifndef SEARCH_CONTROLLER
#define SEARCH_CONTROLLER

#include <random_numbers/random_numbers.h>
#include "Controller.h"
#include "CPFAParameters.h"

/**
 * This class implements the search control algorithm for the rovers. The code
 * here should be modified and enhanced to improve search performance.
 */
class SearchController : virtual Controller {

public:

  SearchController();

  void Reset() override;

  // performs search pattern
  Result DoWork() override;
  bool ShouldInterrupt() override;
  bool HasWork() override;

  void SetCurrentLocation(Point currentLocation);
  void SetCenterLocation(Point centerLocation);
  void SetSuccesfullPickup();
  void SetCurrentTimeInMilliSecs( long int time );

  void setSearchType(bool informed_search);

protected:

  void ProcessData();

private:

  bool giveUpSearching();

  random_numbers::RandomNumberGenerator* rng;
  Point currentLocation;
  Point centerLocation;
  Point searchLocation;
  int attemptCount = 0;
  //struct for returning data to ROS adapter
  Result result;

  // Search state
  // Flag to allow special behaviour for the first waypoint
  bool succesfullPickup = false;
  bool informed_search = false;
  float search_step_size = 0.5;

  long int current_time = 0;
  long int informed_search_time = 0;

  CPFAParameters CPFA_parameters;
};

#endif /* SEARCH_CONTROLLER */
