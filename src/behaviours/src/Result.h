#ifndef RESULT_H
#define RESULT_H
// Defines the Result type which used to standardize the information that
// can be returned by a controller object.

/* EXAMPLE:
 * 
 * This struct returns the neccesary commands to logic controller for the current controller to control the robot
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

#include <vector>

#include "Point.h"
#include <iostream>

using namespace std;

/**
 * The CPFAState enum contains all possible CPFA States for rover to be in
 */
enum CPFAState
{
  start_state,
  set_target_location,
  travel_to_search_site,//2
  reach_search_site,
  switched_to_search,
  search_with_uninformed_walk,
  search_with_informed_walk,
  sense_local_resource_density,
  avoid_obstacle,//8
  return_to_nest,//9
  reached_nest//10
};

/**
 *  The CPFASearchType enum contains all possible travel states for the rover to be in
 */
enum CPFASearchType
{
  random_search,
  site_fidelity,
  pheromone
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
  behavior, //result contains behaviour related signals for logic controller to interpret
  waypoint, //result contains waypoints for drive controller
  precisionDriving //controller wants direct error input into drive controller
};

enum BehaviorTrigger {
  wait, //used by logic controller to indicate to ROSAdapter indicate when nothing should happen
  prevProcess,
  noChange, //guard used by logic controller against faulty configurations
  nextProcess, // deprecated, will be removed later
  COMPLETED,
  FAILED //giveup search and return to nest
};

struct PrecisionDriving {
  float cmdVel = 0.0; //velocity command
  float cmdAngularError = 0.0; //for the current error
  float cmdAngular = 0.0; //for const pid, angular target speed
  float setPointVel = 0.0; //set this to the target speed 
  float setPointYaw = 0.0; //set this to either the target heading or 0

  float left = 0.0; //this is used by drive controller to pass PWM to ROSAdapter
  float right = 0.0;
};

struct Waypoints {
  vector<Point> waypoints;
};

struct Result {
  CPFAState cpfa_state;
  CPFASearchType cpfa_search_type;
  ResultType type; //stores the type of the result

  BehaviorTrigger b; //hold the behaviour type
  Waypoints wpts; //hold the waypoints
  PrecisionDriving pd; //holds precision commands

  float fingerAngle = -1; //holds commanded for finger angle, defualt is -1 no movment
  float wristAngle = -1; //"                  " wrist angle, "                        "
  PIDType PIDMode; //hold the PID type selected for use

  bool reset; //holds a reset command where logic controller will reset the controller that asks
  bool lay_pheromone = false;
};

#endif
