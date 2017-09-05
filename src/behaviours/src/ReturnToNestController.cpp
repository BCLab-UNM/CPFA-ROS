#include "ReturnToNestController.h"

using namespace std;

ReturnToNestController::ReturnToNestController()
{
  result.wristAngle = 0.7;
}

ReturnToNestController::~ReturnToNestController()
{

}

Result ReturnToNestController::DoWork()
{
  double distanceToCenter = hypot(this->centerLocation.x - this->currentLocation.x, this->centerLocation.y - this->currentLocation.y);
  
  cout << "working RTN controller " << endl;

    long int elapsed = current_time - returnTimer;
    timerTimeElapsed = elapsed/1e3; // Convert from milliseconds to seconds

  if (reached_collection_point)
  {
    result.type = behavior;
    result.b = COMPLETED;
    result.reset = true;
    cout << "reached collection point RTN" << endl;
  }
  //check to see if we are driving to the center location or if we need to drive in a circle and look.
  else if ((distanceToCenter > collectionPointVisualDistance && !circular_center_searching)) 
  {

    result.type = waypoint;
    result.wpts.waypoints.clear();
    result.wpts.waypoints.push_back(this->centerLocation);
    start_waypoint = false;
    
    cout << "RTN drive to center" << endl;
    
    return result;

  }
  else if (timerTimeElapsed >= 2)//spin search for center
  {
    Point nextSpinPoint;

    //sets a goal that is 60cm from the centerLocation and spinner
    //radians counterclockwise from being purly along the x-axis.
    nextSpinPoint.x = centerLocation.x + (initialSpinSize + spinSizeIncrease) * cos(spinner);
    nextSpinPoint.y = centerLocation.y + (initialSpinSize + spinSizeIncrease) * sin(spinner);
    nextSpinPoint.theta = atan2(nextSpinPoint.y - currentLocation.y, nextSpinPoint.x - currentLocation.x);

    result.type = waypoint;
    result.wpts.waypoints.clear();
    result.wpts.waypoints.push_back(nextSpinPoint);

    spinner += 45*(M_PI/180); //add 45 degrees in radians to spinner.
    if (spinner > 2*M_PI) {
      spinner -= 2*M_PI;
    }
    spinSizeIncrease += spinSizeIncrement/8;
    circular_center_searching = true;
    //safety flag to prevent us trying to drive back to the
    //center since we have a block with us and the above point is
    //greater than collectionPointVisualDistance from the center.

    returnTimer = current_time;
    timerTimeElapsed = 0;
    
    cout << "spin search RTN" << endl;

  }

  return result;
}

void ReturnToNestController::ProcessData()
{
  long int elapsed = current_time - returnTimer;
  timerTimeElapsed = elapsed/1e3; // Convert from milliseconds to seconds
  cout << "procces data return to nest" << endl;
}

bool ReturnToNestController::ShouldInterrupt() {
  ProcessData();

  bool should_interrupt = false;

  if (start_waypoint && !interrupt && target_held) {
    interrupt = true;
    should_interrupt = true;
    cout << " interupt from RTN 1" << endl;
  }
  else if (reached_collection_point)
  {
    should_interrupt = true;
    cout << "reach collection interupt" << endl;
  }

  return should_interrupt;
}

bool ReturnToNestController::HasWork()
{
  bool has_work = false;
  cout << "asked for work";
  if (timerTimeElapsed >= 2 && target_held)
  {
    cout << "has work RT N" << endl; 
    has_work = true;
  }
  
  return has_work;
}

void ReturnToNestController::SetTargetData(vector<TagPoint> tags) {

  // if a target is detected and we are looking for center tags
  if (tags.size() > 0 && !reached_collection_point) {
    for (int i = 0; i < tags.size(); i++) {
      if (tags[i].id == 256) {
        reached_collection_point = true;
        break;
      }
    }
  }
}

void ReturnToNestController::Reset()
{
  reached_collection_point = false;
  interrupt = false;
  spinSizeIncrease = 0;
}
