#ifndef LOGICCONTROLLER_H
#define LOGICCONTROLLER_H

#include "Controller.h"
#include "PickUpController.h"
#include "DropOffController.h"
#include "SearchController.h"
#include "ObstacleController.h"
#include "DriveController.h"
#include "SiteFidelityController.h"
#include "PheromoneController.h"
#include "RandomDispersalController.h"
#include "CPFAParameters.h"
#include "RangeController.h"
#include "ManualWaypointController.h"

#include <vector>
#include <queue>
#include <stdlib.h>
#include <random_numbers/random_numbers.h>

using namespace std;

// This struct contains a controller object and ties it to a priority value as
// well as providing functionality to compare priorities with the < operator.
struct PrioritizedController {
  int priority = -1;
  Controller* controller = nullptr;

  PrioritizedController(int pri, Controller* cntrl) :
    priority(pri),
    controller(cntrl)
  {
  }

  inline bool operator <(const PrioritizedController& other) const
  {
    return priority < other.priority;
  }
};

class LogicController : virtual Controller
{
public:
  LogicController();
  ~LogicController();

  void Reset() override;
  Result DoWork() override;
  void UpdateData();
  bool ShouldInterrupt() override;
  bool HasWork() override;

  // Give the controller a list of visible april tags.
  // NOTE: This function may be named SetTagData() in other classes
  //       but they are the same function.
  void SetAprilTags(vector<Tag> tags);
  void SetSonarData(float left, float center, float right);
  void SetPositionData(Point currentLocation);
  void SetMapPositionData(Point currentLocationMap);
  void SetVelocityData(float linearVelocity, float angularVelocity);
  void SetMapVelocityData(float linearVelocity, float angularVelocity);
  void SetCenterLocationOdom(Point centerLocationOdom);
  void SetCenterLocationMap(Point centerLocationMap);
  void SetRoverInitLocation(Point location);
  void SetArenaSize(int size);
  
  int getCollisionCalls();
  // Passthrough for providing new waypoints to the
  // ManualWaypointController.
  void AddManualWaypoint(Point wpt, int waypoint_id);

  // Passthrough for removing waypoints from the
  // ManualWaypointController.
  void RemoveManualWaypoint(int waypoint_id);

  // Passthrough for getting the list of manual waypoints that have
  std::vector<int> GetClearedWaypoints();

  // Put the logic controller into manual mode. Changes process state
  // to PROCESS_STATE_MANUAL and logic state to LOGIC_STATE_INTERRUPT.
  // If the logic controller is already in manual mode this has no
  // effect.
  void SetModeManual();

  // Put the logic controller into autonomous mode. Resets the logic
  // controller and clears all manual waypoints.
  //
  // If the logic controller is already in autonomous mode, then this
  // has no effect.
  void SetModeAuto();

  void SetCurrentTimeInMilliSecs( long int time );
  void InsertPheromone(const std::vector<Point>& pheromone_trail); //qilu 12/2017
  void SetCPFAState(CPFAState state) override;
  CPFAState GetCPFAState() override;

  // Tell the logic controller whether rovers should automatically
  // resstrict their foraging range. If so provide the shape of the
  // allowed range.
  void setVirtualFenceOn( RangeShape* range );
  void setVirtualFenceOff( );
  void senseLocalResourceDensity(int num_tags);
  void printCPFAState();
  void printCPFASearchType();
  
  bool layPheromone(); //qilu 12/2017
  //Point getTargetLocation(); //qilu 12/2017
  Point GetCurrentLocation(); //qilu 12/2017
  void UpdatePheromoneList();
  
protected:
  void ProcessData();

private:

  enum LogicState {
    LOGIC_STATE_INTERRUPT = 0,
    LOGIC_STATE_WAITING,
    LOGIC_STATE_PRECISION_COMMAND
  };

  enum ProcessState {
    _FIRST = 0,
    PROCCESS_STATE_SEARCHING = 0,
    PROCCESS_STATE_TARGET_PICKEDUP,
    PROCCESS_STATE_RETURN_NEST,
    PROCCESS_STATE_DROP_OFF,
    PROCESS_STATE_SITE_FIDELITY,
    PROCESS_STATE_PHEROMONE,
    _LAST,
    PROCCESS_STATE_MANUAL
  };
     

  CPFAState cpfa_state = start_state;
  CPFASearchType cpfa_search_type = random_search;
  
  
  LogicState logicState;
  ProcessState processState;

  PickUpController pickUpController;
  DropOffController dropOffController;
  SearchController searchController;
  ObstacleController obstacleController;
  DriveController driveController;
  SiteFidelityController siteFidelityController;
  PheromoneController pheromoneController;  
  RandomDispersalController randomDispersalController;  
  RangeController rangeController;  
  ManualWaypointController manualWaypointController;
  vector<PrioritizedController> prioritizedControllers;
  priority_queue<PrioritizedController> control_queue;

  void controllerInterconnect();
  double getPoissonCDF(const double lambda);
  
  
  long int current_time = 0;
  bool informed_search = false;
  int local_resource_density = 0;
  bool lay_pheromone = false;
 
// CPFA Parameters
  //double probability_of_switching_to_searching = 0.015;
  //double rate_of_following_site_fidelity = 5;//original is 0.3; range [0, 20]; 0 -> 1; 20 -> 0
  //double rate_of_laying_pheromone =5; //range [0, 20]; 0 -> 1; 20 -> 0
  //double rate_of_pheromone_decay = 0.025; //range [0, 1];

  bool first_waypoint = true;
  
  bool set_informed_type =false; //qilu 12/2017
  random_numbers::RandomNumberGenerator* rng;
  
  /* CPFA State Machine variables */
  /*bool target_held = false;
  bool target_dropped_off = true;
  bool nest_has_been_reached = false;
  bool num_pheromones = 0;
  bool informed_search = false;
  int local_resource_density = 0;

  CPFAParameters CPFA_parameters;*/ //remove this block if the code works. 
  /* End CPFA State variables */
CPFAParameters CPFA_parameters;
};

#endif // LOGICCONTROLLER_H

