#ifndef LOGICCONTROLLER_H
#define LOGICCONTROLLER_H

#include "Controller.h"
#include "PickUpController.h"
#include "DropOffController.h"
#include "SearchController.h"
#include "ObstacleController.h"
#include "DriveController.h"
#include "SiteFidelityController.h"
#include "RandomDispersalController.h"
#include "CPFAParameters.h"
#include "RangeController.h"
#include "ManualWaypointController.h"

#include <vector>
#include <queue>
#include <stdlib.h>

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

  void SetAprilTags(std::vector<Tag> tags);
  void SetSonarData(float left, float center, float right);
  void SetPositionData(Point currentLocation);
  void SetMapPositionData(Point currentLocationMap);
  void SetVelocityData(float linearVelocity, float angularVelocity);
  void SetMapVelocityData(float linearVelocity, float angularVelocity);
  void SetCenterLocationOdom(Point centerLocationOdom);
  void SetCenterLocationMap(Point centerLocationMap);
  void AddManualWaypoint(Point wpt, int waypoint_id);
  void RemoveManualWaypoint(int waypoint_id);
  std::vector<int> GetClearedWaypoints();

  void SetModeManual();
  void SetModeAuto();

  void SetCurrentTimeInMilliSecs( long int time );

  //void setTargetHeld();
  // Tell the logic controller whether rovers should automatically
  // resstrict their foraging range. If so provide the shape of the
  // allowed range.
  void setVirtualFenceOn( RangeShape* range );
  void setVirtualFenceOff( );

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
    PROCCESS_STATE_DROP_OFF,
    _LAST,
    PROCCESS_STATE_MANUAL
  };

  LogicState logicState;
  ProcessState processState;

  PickUpController pickUpController;
  DropOffController dropOffController;
  SearchController searchController;
  ObstacleController obstacleController;
  DriveController driveController;
  RangeController range_controller;
  ManualWaypointController manualWaypointController;

  std::vector<PrioritizedController> prioritizedControllers;
  std::priority_queue<PrioritizedController> control_queue;

  void controllerInterconnect();

  long int current_time = 0;

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
