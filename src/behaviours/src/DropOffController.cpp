#include "DropOffController.h"

using namespace std;

DropOffController::DropOffController() {

  result.type = behavior;
  result.b = wait;
  result.wristAngle = 0.7;
  result.reset = false;
  interrupt = false;

 // circularCenterSearching = false;
  spinner = 0;
  centerApproach = false;
  seenEnoughCenterTags = false;
  prevCount = 0;

  countLeft = 0;
  countRight = 0;

  isPrecisionDriving = false;
  timerTimeElapsed = -1;

}

DropOffController::~DropOffController() {

}

Result DropOffController::DoWork() {

  cout << "8" << endl;

  int count = countLeft + countRight;

  if(timerTimeElapsed > -1) {

    long int elapsed = current_time - returnTimer;
    timerTimeElapsed = elapsed/1e3; // Convert from milliseconds to seconds
  }

  //if we are in the routine for exiting the circle once we have dropped a block off and reseting all our flags
  //to resart our search.
  if(false)
  {
    cout << "2" << endl;
    if (timerTimeElapsed >= 5)
    {
      if (finalInterrupt)
      {
        result.type = behavior;
        result.b = nextProcess;
        result.reset = true;
        return result;
      }
      else
      {
        finalInterrupt = true;
        cout << "1" << endl;
      }
    }
    else if (timerTimeElapsed >= 0.1)
    {
      isPrecisionDriving = true;
      result.type = precisionDriving;

      result.fingerAngle = M_PI_2; //open fingers
      result.wristAngle = 0; //raise wrist

      result.pd.cmdVel = -0.3;
      result.pd.cmdAngularError = 0.0;
    }

    return result;
  }



  bool left = (countLeft > 0);
  bool right = (countRight > 0);
  bool centerSeen = (right || left);

  //reset lastCenterTagThresholdTime timout timer to current time
  if ((!centerApproach && !seenEnoughCenterTags) || (count > 0 && !seenEnoughCenterTags)) {

    lastCenterTagThresholdTime = current_time;

  }

  if (count > 0 || seenEnoughCenterTags || prevCount > 0) //if we have a target and the center is located drive towards it.
  {

    cout << "9" << endl;
    centerSeen = true;

    if (first_center && isPrecisionDriving)
    {
      first_center = false;
      result.type = behavior;
      result.reset = false;
      result.b = nextProcess;
      return result;
    }
    isPrecisionDriving = true;

    if (seenEnoughCenterTags) //if we have seen enough tags
    {
      if ((countLeft-5) > countRight) //and there are too many on the left
      {
        right = false; //then we say none on the right to cause us to turn right
      }
      else if ((countRight-5) > countLeft)
      {
        left = false; //or left in this case
      }
    }

    float turnDirection = 1;
    float current_search_velocity = searchVelocity;
    
    //reverse tag rejection when we have seen enough tags that we are on a
    //trajectory in to the square we dont want to follow an edge.
    //increase turning power to more strongly turn inwards or on occasion outwards
    //drive more slowly so turning radius is smaller
    if (seenEnoughCenterTags) 
    {
      turnDirection = -4;
      current_search_velocity = 0.05;
    }

    result.type = precisionDriving;

    //otherwise turn till tags on both sides of image then drive straight
    if (!(left ^ right)) {
      result.pd.cmdVel = current_search_velocity;
      result.pd.cmdAngularError = 0.0;
    }
    else if (right) {
      result.pd.cmdVel = -0.1 * turnDirection;
      result.pd.cmdAngularError = -centeringTurnRate*turnDirection;
    }
    else if (left){
      result.pd.cmdVel = -0.1 * turnDirection;
      result.pd.cmdAngularError = centeringTurnRate*turnDirection;
    }

    //must see greater than this many tags before assuming we are driving into the center and not along an edge.
    if (count > centerTagThreshold)
    {
      seenEnoughCenterTags = true; //we have driven far enough forward to be in and aligned with the circle.
      lastCenterTagThresholdTime = current_time;
    }
    if (count > 0) //reset gaurd to prevent drop offs due to loosing tracking on tags for a frame or 2.
    {
      lastCenterTagThresholdTime = current_time;
    }
    //time since we dropped below countGuard tags
    long int elapsed = current_time - lastCenterTagThresholdTime;
    float timeSinceSeeingEnoughCenterTags = elapsed/1e3; // Convert from milliseconds to seconds

    //we have driven far enough forward to have passed over the circle.
    if (count < 1 && seenEnoughCenterTags && timeSinceSeeingEnoughCenterTags > dropDelay) {
      centerSeen = false;
    }
    centerApproach = true;
    prevCount = count;
    count = 0;
    countLeft = 0;
    countRight = 0;
  }

  //was on approach to center and did not seenEnoughCenterTags
  //for lostCenterCutoff seconds so reset.
  else if (centerApproach) {

    long int elapsed = current_time - lastCenterTagThresholdTime;
    float timeSinceSeeingEnoughCenterTags = elapsed/1e3; // Convert from milliseconds to seconds
    if (timeSinceSeeingEnoughCenterTags > lostCenterCutoff)
    {
      cout << "4" << endl;
      //go back to drive to center base location instead of drop off attempt
      //reachedCollectionPoint = false;
      seenEnoughCenterTags = false;
      centerApproach = false;

      result.type = waypoint;
      result.wpts.waypoints.push_back(this->centerLocation);
      if (isPrecisionDriving) {
        result.type = behavior;
        result.b = prevProcess;
        result.reset = false;
      }
      isPrecisionDriving = false;
      interrupt = false;
      precisionInterrupt = false;
    }
    else
    {
      result.pd.cmdVel = searchVelocity;
      result.pd.cmdAngularError = 0.0;
    }

    return result;

  }

  if (!centerSeen && seenEnoughCenterTags)
  {
    //reachedCollectionPoint = true;
    centerApproach = false;
    returnTimer = current_time;
  }

  return result;
}

void DropOffController::Reset() {
  result.type = behavior;
  result.b = wait;
  result.pd.cmdVel = 0;
  result.pd.cmdAngularError = 0;
  result.fingerAngle = -1;
  result.wristAngle = 0.7;
  result.reset = false;
  result.wpts.waypoints.clear();
  spinner = 0;
  spinSizeIncrease = 0;
  prevCount = 0;
  timerTimeElapsed = -1;

  countLeft = 0;
  countRight = 0;


  //reset flags
  //reachedCollectionPoint = false;
  seenEnoughCenterTags = false;
  isPrecisionDriving = false;
  finalInterrupt = false;
  precisionInterrupt = false;
  target_held = false;
  first_center = true;

}

void DropOffController::SetTargetData(vector<TagPoint> tags) {
  countRight = 0;
  countLeft = 0;

  if(target_held) {
    // if a target is detected and we are looking for center tags
    if (tags.size() > 0 /*&& !reachedCollectionPoint*/) {

      // this loop is to get the number of center tags
      for (int i = 0; i < tags.size(); i++) {
        if (tags[i].id == 256) {

          // checks if tag is on the right or left side of the image
          if (tags[i].x + cameraOffsetCorrection > 0) {
            countRight++;

          } else {
            countLeft++;
          }
        }
      }
    }
  }

}

void DropOffController::ProcessData() {
  if((countLeft + countRight) > 0) {
    isPrecisionDriving = true;
  }
}

bool DropOffController::ShouldInterrupt() {
  ProcessData();

  if (isPrecisionDriving && !precisionInterrupt) {
    precisionInterrupt = true;
    return true;
  }
  if (finalInterrupt) {
    return true;
  }
}

bool DropOffController::HasWork() {

  if(timerTimeElapsed > -1) {
    long int elapsed = current_time - returnTimer;
    timerTimeElapsed = elapsed/1e3; // Convert from milliseconds to seconds
  }
}

bool DropOffController::IsChangingMode() {
  return isPrecisionDriving;
}

void DropOffController::SetCenterLocation(Point center) {
  centerLocation = center;
}

void DropOffController::SetCurrentLocation(Point current) {
  currentLocation = current;
}

void DropOffController::SetTargetPickedUp() {
  target_held = true;
}

void DropOffController::SetCurrentTimeInMilliSecs( long int time )
{
  current_time = time;
}
