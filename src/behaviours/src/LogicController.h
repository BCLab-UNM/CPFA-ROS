#ifndef LOGICCONTROLLER_H
#define LOGICCONTROLLER_H

#include "Controller.h"
#include "PickUpController.h"
#include "DropOffController.h"
#include "SearchController.h"
#include "ObstacleController.h"
#include "DriveController.h"
#include "SiteFidelityController.h"
#include "CPFAParameters.h"

#include <vector>
#include <queue>
#include <stdlib.h>

enum LogicState {
  LOGIC_STATE_INTERRUPT = 0,
  LOGIC_STATE_WAITING,
  LOGIC_STATE_PRECISION_COMMAND
};

enum ProcessState {
  PROCESS_STATE_START = 0,
  PROCESS_STATE_SITE_FIDELITY,
  PROCESS_STATE_PHEROMONE,
  PROCESS_STATE_RANDOM_DISPERSAL,
  PROCESS_STATE_SEARCHING, // uninformed and informed correlated random walk
  PROCESS_STATE_RETURN_TO_NEST,
  PROCESS_STATE_DROPOFF,
};

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

  void SetAprilTags(std::vector<TagPoint> tags);
  void SetSonarData(float left, float center, float right);
  void SetPositionData(Point currentLocation);
  void SetMapPositionData(Point currentLocationMap);
  void SetVelocityData(float linearVelocity, float angularVelocity);
  void SetMapVelocityData(float linearVelocity, float angularVelocity);
  void SetCenterLocationOdom(Point centerLocationOdom);
  void SetCenterLocationMap(Point centerLocationMap);

  void SetCurrentTimeInMilliSecs( long int time );

protected:
  void ProcessData();

private:

  /* CPFA helper functions */
  ProcessState CPFAStateMachine(ProcessState state);
  double PoissonCDF(double lambda);
  /* End CPFA helper functions */

  LogicState logicState;
  ProcessState processState;

  PickUpController pickUpController;
  DropOffController dropOffController;
  SearchController searchController;
  ObstacleController obstacleController;
  DriveController driveController;
  SiteFidelityController site_fidelity_controller;

  std::vector<PrioritizedController> prioritizedControllers;
  std::priority_queue<PrioritizedController> control_queue;

  void controllerInterconnect();

  long int current_time = 0;

  /* CPFA State Machine variables */
  bool target_held = false;
  bool target_dropped_off = true;
  bool nest_has_been_reached = false;
  bool num_pheromones = 0;
  int local_resource_density = 0;

  CPFAParameters CPFA_parameters;
  /* End CPFA State variables */

};

#endif // LOGICCONTROLLER_H
