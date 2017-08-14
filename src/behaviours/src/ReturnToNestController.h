#ifndef RETURNTONESTCONTROLLER_H
#define RETURNTONESTCONTROLLER_H

#include "Controller.h"
#include "TagPoint.h"
#include <math.h>
#include <vector>

class ReturnToNestController : virtual Controller
{

public:
  
  ReturnToNestController();
  ~ReturnToNestController();

  void Reset() override;
  Result DoWork() override;
  bool ShouldInterrupt() override;
  bool HasWork() override;
  
  void SetTargetData(std::vector<TagPoint> tags);
  void SetCurrentLocation(Point current) {currentLocation = current;}
  void SetCenterLocation(Point center) {centerLocation = center;}
  
  void SetCurrentTimeInMilliSecs( long int time );


private:

  Result result;

  void ProcessData();
  
  //Constants
  
  const float collectionPointVisualDistance = 0.2; //in meters
  const float initialSpinSize = 0.05; //in meters aka 10cm
  const float spinSizeIncrement = 0.50; //in meters
  const float searchVelocity = 0.15; //in meters per second
  
  //The amount over initialSpinSize we've gotten to
  float spinSizeIncrease;
  
  //keep track of progression around a circle when driving in a circle
  float spinner;
  
  //Timer for return code (dropping the cube in the center)- used for timerTimeElapsed
  long int returnTimer;
  
  //Time since modeTimer was started, in seconds
  float timerTimeElapsed;
  
  //Center and current locations as of the last call to setLocationData
  Point centerLocation;
  Point currentLocation;
  
  long int current_time;
  


  /*
     *  Flags
     */

  //Flag indicating that a target has been picked up and is held
  bool target_held;

  //Flag indicating that we're in the center
  bool reached_collection_point;

  //Flag indicating that we're driving in a circle to find the nest
  bool circular_center_searching;

  //Flag to indicate that we're starting to follow waypoints
  bool start_waypoint = true;

  bool interrupt = false;


};

#endif // RETURNTONESTCONTROLLER_H
