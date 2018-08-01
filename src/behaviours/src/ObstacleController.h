#ifndef OBSTACLECONTOLLER_H
#define OBSTACLECONTOLLER_H

#include "Controller.h"
#include "Tag.h"
#include <random_numbers/random_numbers.h>

class ObstacleController : virtual Controller
{
public:
  ObstacleController();

  Result result;

  void Reset() override;
  Result DoWork() override;
  void SetSonarData(float left, float center, float right);
  void SetCurrentLocation(Point currentLocation);
  void SetTagData(vector<Tag> tags);
  bool ShouldInterrupt() override;
  bool HasWork() override;
  void SetIgnoreCenterSonar();
  void SetCurrentTimeInMilliSecs( long int time );
  void SetTargetHeld();

  // Checks if a target is held and if so resets the state of the obestacle controller otherwise does nothing
  //Asked by logiccontroller to determine if drive controller should have its waypoints cleared
  void SetTargetHeldClear();
  bool GetShouldClearWaypoints() {bool tmp = clearWaypoints; clearWaypoints = false; return tmp;}
  void SetCPFAState(CPFAState state) override;
  CPFAState GetCPFAState() override;
  
protected:

  void ProcessData();

private:

  // Try not to run over the collection zone
  void avoidCollectionZone();

  // Try not to run into a physical object
  void avoidObstacle();

  // Are there AprilTags in the camera view that mark the collection zone
  // and are those AprilTags oriented towards or away from the camera.
  bool checkForCollectionZoneTags( vector<Tag> );
  
  const float K_angular = 0.8; //1.2; //radians a second turn rate to avoid obstacles
  const float reactivate_center_sonar_threshold = 0.5;  //origin is 0.8; reactive center sonar if it goes back above this distance, assuming it is deactivated
  const int targetCountPivot = 6; ///unused variable
  const float obstacleDistancePivot = 0.2526; ///unused variable
  const float triggerDistance = 0.5; //origin is 0.8

  /*
     * Member variables
     */

  random_numbers::RandomNumberGenerator* rng;
  
  bool obstacleInterrupt; //records if obstacle has interupted
  bool obstacleDetected;  //records if an obstacle has been detected
  bool obstacleAvoided; //record if an obstacke has been avoided
  bool clearWaypoints = false;  //record if drivecontrollers waypoints should be cleared

  float left = 0; //distance on left ultrasound
  float center = 0; //distance on center ultrasound
  float right = 0; //distance on right ultrasound
  
  float pitches = 0.0;

  unsigned int count_left_collection_zone_tags;
  unsigned int count_right_collection_zone_tags;

  // Ignore the center sonar because we are carrying a target
  bool ignore_center_sonar = false;

  Point currentLocation;

  long int current_time;
  long int timeSinceTags;
  long int delay;

  bool targetHeld = false;
  bool previousTargetState = false;

  bool phys = false; // Physical obstacle
  bool collection_zone_seen = false; // The obstacle is the collection zone
  
  bool set_waypoint = false;
  bool can_set_waypoint = false;
  CPFAState cpfa_state = start_state;

  float camera_offset_correction = 0.020; //meters;
};

#endif // OBSTACLECONTOLLER_H
