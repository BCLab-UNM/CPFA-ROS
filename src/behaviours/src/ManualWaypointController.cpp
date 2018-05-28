#include <map>

#include "ManualWaypointController.h"
#include <angles/angles.h> // for hypot()

ManualWaypointController::ManualWaypointController() {}

ManualWaypointController::~ManualWaypointController() {}

void ManualWaypointController::Reset() {
  waypoints.clear();
  num_waypoints = 0;
  cleared_waypoints.clear();
}
	
bool ManualWaypointController::HasWork() {
	//cout<<"Manual waypoint has work..."<<!waypoints.empty()<<endl;
  return !waypoints.empty();
}

bool ManualWaypointController::ShouldInterrupt() {
	//cout<<"manualwaypoint controller should interrupt...";
  bool interrupt = false;
  // If the size of the manual waypoint list has changed, then interrupt.
  if(num_waypoints != waypoints.size() && !waypoints.empty()) {
    interrupt = true;
    num_waypoints = waypoints.size();
  }
  cout<<interrupt<<endl;
  return interrupt;
}

Result ManualWaypointController::DoWork() {
	//cout<<"ManualWaypointController::DoWork()"<<endl;
  Result result;
  result.type = waypoint;
  result.wpts.waypoints.push_back(waypoints.begin()->second);
  result.PIDMode = FAST_PID;
  return result;
}

void ManualWaypointController::SetCurrentLocation(Point currentLocation)
{
  this->currentLocation = currentLocation;
  if(!waypoints.empty()) {
    std::map<int, Point>::iterator first = waypoints.begin();
    if(hypot(first->second.x-currentLocation.x,
             first->second.y-currentLocation.y)
       < waypoint_tolerance) {
      cleared_waypoints.push_back(first->first);
      waypoints.erase(first);
    }
  }
}

void ManualWaypointController::ProcessData()
{   
}

CPFAState ManualWaypointController::GetCPFAState() 
{
  return cpfa_state;
}

void ManualWaypointController::SetCPFAState(CPFAState state) {
  cpfa_state = state;
  result.cpfa_state = state;
}

void ManualWaypointController::AddManualWaypoint(Point wpt, int id)
{
  waypoints[id] = wpt;
}

void ManualWaypointController::RemoveManualWaypoint(int id)
{
  waypoints.erase(id);
}

std::vector<int> ManualWaypointController::ReachedWaypoints() {
  std::vector<int> cleared = cleared_waypoints;
  cleared_waypoints.clear();
  return cleared;
}

