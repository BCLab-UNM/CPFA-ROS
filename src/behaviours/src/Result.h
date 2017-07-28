#ifndef RESULT_H
#define RESULT_H

// Defines the Result type which used to standardize the information that
// can be returned by a controller object.

/* EXAMPLE:
 *
 * struct Result res;
 * res.type = ...;
 * res.b = ...;
 * ...
 *
 * if(res.type == behavior) {
 *      if(res.b == targetDropped) {
 *          ...
 *      } else if(res.b == ...) {
 *          ...
 *      } ...
 * } else if(res.type == waypoint) {
 *      Pose2D next = res.wpts.waypoint[0];
 *      ...
 * } else if(res.type == precisionDriving) {
 *      ...
 * }
 *
 *
 *
 */

#include <iostream>
#include <vector>

#include "Point.h"

using namespace std;

/**
 * The CPFAState enum contains all possible CPFA States for rover to be in
 */
enum CPFAState
{
  start_state,
  set_target_location,
  travel_to_search_site,
  search_with_uninformed_walk,
  search_with_informed_walk,
  sense_local_resource_density,
  return_to_nest
};

/**
 *  The CPFASearchType enum contains all possible travel states for the rover to be in
 */
enum CPFASearchType
{
  site_fidelity,
  pheromone,
  random_search
};

/**
 * This class implements the search control algorithm for the rovers. The code
 * here should be modified and enhanced to improve search performance.
 *
 * The search controller is implemented with the CPFA.
 * (Central Place Foraging Algorithm)
 */

enum PIDType {
  FAST_PID, //quickest turn reasponse time
  SLOW_PID, //slower turn reasponse time
  CONST_PID //constant angular turn rate
};

enum ResultType {
  behavior,
  waypoint,
  precisionDriving
};

enum BehaviorTrigger {
  wait,
  prevProcess,
  noChange,
  nextProcess
};

struct PrecisionDriving {
  float cmdVel = 0.0;
  float cmdAngularError = 0.0;
  float cmdAngular = 0.0;
  float setPointVel = 0.0;
  float setPointYaw = 0.0;

  float left = 0.0;
  float right = 0.0;
};

struct Waypoints {
  vector<Point> waypoints;
};

struct Result {
  CPFAState cpfa_state;
  CPFASearchType cpfa_search_type;

  ResultType type;

  BehaviorTrigger b;
  Waypoints wpts;
  PrecisionDriving pd;

  float fingerAngle = -1;
  float wristAngle = -1;
  PIDType PIDMode;

  bool reset;
};

#endif 
