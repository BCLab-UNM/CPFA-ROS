#ifndef LOGICCONTROLLER_H
#define LOGICCONTROLLER_H

#include "Controller.h"
#include "PickUpController.h"
#include "DropOffController.h"
#include "CPFASearchController.h"
#include "ObstacleController.h"
#include "DriveController.h"
#include "Result.h"

#include <vector>
#include <queue>

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

  void SetAprilTags(vector<TagPoint> tags);
  void SetSonarData(float left, float center, float right);
  void SetPositionData(Point currentLocation);
  void SetMapPositionData(Point currentLocationMap);
  void SetVelocityData(float linearVelocity, float angularVelocity);
  void SetMapVelocityData(float linearVelocity, float angularVelocity);
  void SetCenterLocationOdom(Point centerLocationOdom);
  void SetCenterLocationMap(Point centerLocationMap);
  void SetCurrentTimeInMilliSecs( long int time );
  void SetArenaSize(int numRovers);
  void insertPheromone(const std::vector<Point>& pheromone_trail);

  void SetCPFAState(CPFAState state) override;
  CPFAState GetCPFAState() override;

  void SetCPFASearchType(CPFASearchType search_type) override;
  CPFASearchType GetCPFASearchType() override;

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
    _LAST
  };

  CPFAState cpfa_state = start_state;
  CPFASearchType cpfa_search_type = random_search;

  LogicState logicState;
  ProcessState processState;

  PickUpController pickUpController = PickUpController("PickUpController");
  DropOffController dropOffController = DropOffController("dropOffController");
  CPFASearchController searchController = CPFASearchController("CPFASearchController");
  ObstacleController obstacleController = ObstacleController("ObstacleController");
  DriveController driveController = DriveController("DriveController");

  std::vector<PrioritizedController> prioritizedControllers;
  priority_queue<PrioritizedController> control_queue;

  void controllerInterconnect();

  long int current_time = 0;
};

#endif // LOGICCONTROLLER_H
