#include "ObstacleController.h"

ObstacleController::ObstacleController(std::string name)
{
  this->name = name;
  obstacleAvoided = true;
  obstacleDetected = false;
  obstacleInterrupt = false;
  result.PIDMode = CONST_PID;
}

void ObstacleController::Reset() {
  obstacleAvoided = true;
  obstacleDetected = false;
  obstacleInterrupt = false;
}

Result ObstacleController::DoWork() {
  clearWaypoints = true;

  if(centerSeen){

    result.type = precisionDriving;

    result.pd.cmdVel = 0.0;

    //cout << "countLeft: " << countLeft << " countRight: " << countRight << endl;
    //if(countLeft < countRight) {
      //result.pd.cmdAngular = K_angular;
      //turn_direction = true; // counterclockwise
    //} else {
      //result.pd.cmdAngular = -K_angular;
      //turn_direction = false; // clockwise
    //}

    result.pd.cmdAngular = 0;
    result.pd.setPointVel = 0.0;
    result.pd.cmdVel = -0.3;
    result.pd.setPointYaw = 0;
    countLeft = 0;
    countRight = 0;

  }
  else {

    //obstacle on right side
    if (right < 0.8 || center < 0.8 || left < 0.8) {
      result.type = precisionDriving;

      //if (right < left)
      //{
        //result.pd.cmdAngular = K_angular;
        //turn_direction = true; // clockwise
      //} else 
      //{
        result.pd.cmdAngular = -K_angular;
        turn_direction = false;
      //}

      result.pd.setPointVel = 0.0;
      result.pd.cmdVel = 0.0;
      result.pd.setPointYaw = 0;
    }
  }

  cout << "turn_direction: ";
  if(turn_direction)
  {
    cout << "counter-clockwise";
  } else 
  {
    cout << "clockwise";
  }
  cout << endl;

  return result;
}


void ObstacleController::SetSonarData(float sonarleft, float sonarcenter, float sonarright) {
  left = sonarleft;
  right = sonarright;
  center = sonarcenter;

  ProcessData();
}

void ObstacleController::SetCurrentLocation(Point currentLocation) {
  this->currentLocation = currentLocation;
}

void ObstacleController::ProcessData() {

  //timeout timer for no tag messages
  long int Tdifference = current_time - timeSinceTags;
  float Td = Tdifference/1e3;
  if (Td >= 0.5) {
    centerSeen = false;
  }

  //Process sonar info
  if(ignoreCenter){
    if(center > reactivateCenterThreshold){
      ignoreCenter = false;
    }
    else{
      center = 3;
    }
  }

  if (left < triggerDistance || right < triggerDistance || center < triggerDistance || centerSeen) {
    obstacleDetected = true;
    obstacleAvoided = false;
  } else {
    obstacleAvoided = true;
  }
}

void ObstacleController::SetTagData(vector<TagPoint> tags){
  float cameraOffsetCorrection = 0.020; //meters;
  centerSeen = false;
  // this loop is to get the number of center tags
  if (!targetHeld) {
    for (int i = 0; i < tags.size(); i++) {
      if (tags[i].id == 256) {

        // checks if tag is on the right or left side of the image
        if (tags[i].x + cameraOffsetCorrection > 0) {
          countRight++;

        } else {
          countLeft++;
        }
        centerSeen = true;
        timeSinceTags = current_time;
      }
    }
  }

}

bool ObstacleController::ShouldInterrupt() {

  if(obstacleDetected && !obstacleInterrupt) {
    obstacleInterrupt = true;
    return true;
  } else {
    if(obstacleAvoided && obstacleDetected) {
      Reset();
      return true;
    } else {
      return false;
    }
  }
}

bool ObstacleController::HasWork() {

  return !obstacleAvoided;
}

void ObstacleController::SetIgnoreCenter(){
  ignoreCenter = true; //ignore center ultrasound
}

void ObstacleController::SetCurrentTimeInMilliSecs( long int time )
{
  current_time = time;
}

void ObstacleController::SetTargetHeld() {
  targetHeld = true;
  if (previousTargetState == false) {
    obstacleAvoided = true;
    obstacleInterrupt = false;
    obstacleDetected = false;
    previousTargetState = true;
  }
}

CPFAState ObstacleController::GetCPFAState() 
{
  return cpfa_state;
}

void ObstacleController::SetCPFAState(CPFAState state) {
  cpfa_state = state;
  result.cpfa_state = state;
}


CPFASearchType ObstacleController::GetCPFASearchType() 
{
  return cpfa_search_type;
}

void ObstacleController::SetCPFASearchType(CPFASearchType type)
{
  cpfa_search_type = type;
  result.cpfa_search_type = type;
}

bool ObstacleController::getTurnDirection() 
{
  return turn_direction;
}
