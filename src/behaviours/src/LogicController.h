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

struct PrioritizedController {
  int priority = -1;
  Controller* controller = nullptr;

  PrioritizedController(int pri, Controller* cntrl) : priority(pri), controller(cntrl) {}

  inline bool operator <(const PrioritizedController& other) const {
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

  void SetAprilTags(vector<Tag> tags);
  void SetSonarData(float left, float center, float right);
  void SetPositionData(Point currentLocation);
  void SetMapPositionData(Point currentLocationMap);
  void SetVelocityData(float linearVelocity, float angularVelocity);
  void SetMapVelocityData(float linearVelocity, float angularVelocity);
  void SetCenterLocationOdom(Point centerLocationOdom);
   //int GetCenterIdx();
  void SetCenterLocationMap(Point centerLocationMap);
  void AddManualWaypoint(Point wpt, int waypoint_id);
  void RemoveManualWaypoint(int waypoint_id);
  std::vector<int> GetClearedWaypoints();

  void SetModeManual();
  void SetModeAuto();

  void SetCurrentTimeInMilliSecs( long int time );
  void insertPheromone(const std::vector<Point>& pheromone_trail); //qilu 12/2017
  void SetCPFAState(CPFAState state) override;
  CPFAState GetCPFAState() override;

  //void SetTargetHeld();
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
  void updatePheromoneList();
  
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
    PROCESS_STATE_SENSE_DENSITY,
    PROCCESS_STATE_DROP_OFF,
    PROCESS_STATE_SITE_FIDELITY,
    PROCESS_STATE_PHEROMONE,
    _LAST,
    PROCCESS_STATE_MANUAL
  };//PROCESS_STATE_SENSE_DENSITY,
     

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
  double probability_of_switching_to_searching = 0.015;
  double probability_of_returning_to_nest=0.001;
  double uninformed_search_variation= 0.4;
  double rate_of_informed_search_decay =0.1666;
  double rate_of_following_site_fidelity = 0.6;//original is 0.3
  double rate_of_laying_pheromone =5;
  double rate_of_pheromone_decay = 0.025;
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

};

#endif // LOGICCONTROLLER_H

