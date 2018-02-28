#include "ObstacleController.h"

using namespace std;

ObstacleController::ObstacleController()
{
  obstacleAvoided = true;
  obstacleDetected = false;
  obstacleInterrupt = false;
  result.PIDMode = CONST_PID; //use the const PID to turn at a constant speed
  rng = new random_numbers::RandomNumberGenerator();
  
  //cout << "ObstacleController -> 0" << endl;

}


//note, not a full reset as this could cause a bad state
//resets the interupt and knowledge of an obstacle or obstacle avoidance only.
void ObstacleController::Reset() {
  obstacleAvoided = true;
  obstacleDetected = false;
  obstacleInterrupt = false;
  delay = current_time;
  SetCPFAState(start_state);
}

// Avoid crashing into objects detected by the ultraound
void ObstacleController::avoidObstacle() {
	cout<<"TestStatusA: avoidObstacle..."<<endl;
 	//cout<<"CollisionStatus: left="<<left<<", center="<<center<<", right="<<right<<endl;
    if (left <= right && left <= center && left <triggerDistance) 
    {  
		//cout<<"CollisionStatus: 1. turn to right"<<endl;
      result.pd.cmdAngular = -K_angular; 
    }
    else if (right < left && right < center && right < triggerDistance) //turn left
    {
		//cout<<"CollisionStatus: 1. turn to left"<<endl;
      result.pd.cmdAngular = K_angular;
    }
    else //the obstacle is in front 
    {
		double p = rng->uniformReal(0, 1.0);
      if(p<=0.5) //turn left
      {
		  //cout<<"CollisionStatus: 2. turn to left"<<endl;
		result.pd.cmdAngular = K_angular;
      }
      else //turn right
      {
		  //cout<<"CollisionStatus: 2. turn to right"<<endl;
        result.pd.cmdAngular = -K_angular;
	  }
    }
    result.type = precisionDriving;
    result.pd.setPointVel = 0.0;
    
    double vel = rng->uniformReal(0.05, 0.2);
    result.pd.cmdVel = vel;
    //result.pd.cmdVel = 0.0;
    result.pd.setPointYaw = 0;
    
}

// A collection zone was seen in front of the rover and we are not carrying a target
// so avoid running over the collection zone and possibly pushing cubes out.
void ObstacleController::avoidCollectionZone() {
 cout<<"TestStatusA: avoid collection zone..."<<endl;
    result.type = precisionDriving;

    //result.pd.cmdVel = 0.0;

    // Decide which side of the rover sees the most april tags and turn away
    // from that side
    /*if(count_left_collection_zone_tags < count_right_collection_zone_tags) {
      result.pd.cmdAngular = K_angular;
    } else {
      result.pd.cmdAngular = -K_angular;
    }*/
    
    if (pitches < 0) //turn to the right
      {
		  result.pd.cmdAngular = -K_angular;
		  //cout<<"CollisionStatus: avoid disk, turn to right"<<endl;
      }
      else //turn to the left
      {
		  result.pd.cmdAngular = K_angular;
		  //cout<<"CollisionStatus: avoid disk, turn to left"<<endl;
      }   
    result.pd.setPointVel = 0.0;
    //result.pd.cmdVel = 0.0;
	double vel = rng->uniformReal(0.05, 0.1);
      
    result.pd.cmdVel = vel; //qilu 02/2018
    result.pd.setPointYaw = 0;
}


Result ObstacleController::DoWork() {
 cout<<"TestStatusA: ObstacleController::DoWork()..."<<endl;
  clearWaypoints = true;
  set_waypoint = true;
  result.PIDMode = CONST_PID;
  // The obstacle is an april tag marking the collection zone
  if(collection_zone_seen){
    avoidCollectionZone();
    //haveAvoidCollectionZone = true;
    //cout<<"TestStatusA: set haveAvoidCollectionZone="<<haveAvoidCollectionZone<<endl;
  }
  else {
    avoidObstacle();
  }
   //cout<<"TestStatusA: haveAvoidCollectionZone="<<haveAvoidCollectionZone<<endl;
  //if an obstacle has been avoided
  if (can_set_waypoint) {
    can_set_waypoint = false; //only one waypoint is set
    set_waypoint = false;
    clearWaypoints = false;

    result.type = waypoint; 
    result.PIDMode = FAST_PID; //use fast pid for waypoints
    Point forward;            //waypoint is directly ahead of current heading
    /*if(haveAvoidCollectionZone)// if seen collection zone in the past and avoid it now, sample a further location
    {
		cout<<"TestStatusA: ****sample a further location..."<<endl;
		double stepSize = rng->uniformReal(1.0, 2.0);
		forward.x = currentLocation.x + (stepSize * cos(currentLocation.theta));
        forward.y = currentLocation.y + (stepSize * sin(currentLocation.theta));
	}
    else
    {*/
		forward.x = currentLocation.x + (0.5 * cos(currentLocation.theta));
        forward.y = currentLocation.y + (0.5 * sin(currentLocation.theta));
    //}
    //haveAvoidCollectionZone = false;
    //cout<<"TestStatusA: obstacleCTRL sampled waypoint=["<<forward.x<<","<<forward.y<<"]"<<endl;
    result.wpts.waypoints.clear();
    result.wpts.waypoints.push_back(forward);
  }

  return result;
}


void ObstacleController::SetSonarData(float sonarleft, float sonarcenter, float sonarright) 
{
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
  //this is used to set collection zone seen to false beacuse
  //there is no report of 0 tags seen
  long int Tdifference = current_time - timeSinceTags;
  float Td = Tdifference/1e3;
  if (Td >= 0.5) {
	  //cout<<"TestStatus: set collection_zone_seen to false; "<<collection_zone_seen<<endl;
    collection_zone_seen = false; 
    phys= false;
    //if (!obstacleAvoided)
    //{
		cout<<"TestStatus: obstacle not avoid..."<<endl;
      can_set_waypoint = true;
    //}
  }

  //If we are ignoring the center sonar
  if(ignore_center_sonar)
  {
    //If the center distance is longer than the reactivation threshold 
    if(center > reactivate_center_sonar_threshold)
    {
      //ignore_center_sonar = false; //look at sonar again beacuse center ultrasound has gone long
      //ignore_center_sonar = false; //look at sonar again beacuse center ultrasound has gone long
    }
    else
    {
      //set the center distance to "max" to simulated no obstacle
      center = 3;
    }
  }
  else 
  {
    //this code is to protect against a held block causing a false short distance
    //currently pointless due to above code
    if (center < 3.0) {
      result.wristAngle = 0.7;
    }
    else {
      result.wristAngle = -1;
    }
  }

  //if any sonar is below the trigger distance set physical obstacle true
  if (left < triggerDistance || right < triggerDistance || center < triggerDistance)
  {
    phys = true;
   //cout<<"SwitchStatus: detect an obstacle..."<<endl;
    timeSinceTags = current_time;
  }

  //if physical obstacle or collection zone visible
  if (collection_zone_seen || phys)
  {
    obstacleDetected = true;
    obstacleAvoided = false;
    can_set_waypoint = false;
  }
  else
  {
    obstacleAvoided = true;
  }
}

// Report April tags seen by the rovers camera so it can avoid
// the collection zone
// Added relative pose information so we know whether the
// top of the AprilTag is pointing towards the rover or away.
// If the top of the tags are away from the rover then treat them as obstacles. 
void ObstacleController::SetTagData(vector<Tag> tags){
	//cout<<"TestStatus: setTagData: set collection_zone_seen to false"<<endl;
  collection_zone_seen = false;
  count_left_collection_zone_tags = 0;
  count_right_collection_zone_tags = 0;
  pitches =0.0;
  
  // this loop is to get the number of center tags
  if (!targetHeld) 
  {
    for (int i = 0; i < tags.size(); i++) 
	{ //redundant for loop
      if (tags[i].getID() == 256) 
	  {
     	collection_zone_seen = checkForCollectionZoneTags( tags );
     	cout<<"TestStatus: detect collection disk --"<<collection_zone_seen<<endl;
        timeSinceTags = current_time;
      }
    }
  }
}

bool ObstacleController::checkForCollectionZoneTags( vector<Tag> tags ) {

  pitches =0.0;
  for ( auto & tag : tags ) { 

    // Check the orientation of the tag. If we are outside the collection zone the yaw will be positive so treat the collection zone as an obstacle. 
    //If the yaw is negative the robot is inside the collection zone and the boundary should not be treated as an obstacle. 
    //This allows the robot to leave the collection zone after dropping off a target.
    if ( tag.calcYaw() > 0 ) 
    {
	  // checks if tag is on the right or left side of the image
	  if (tag.getPositionX() + camera_offset_correction > 0) 
	  {
	    count_right_collection_zone_tags++;
	  } 
	  else 
	  {
	    count_left_collection_zone_tags++;
	  }
    }
    //check the pitch of detected tags
    
    pitches += tag.calcPitch();
    
  }
  pitches /= tags.size();
      


  // Did any tags indicate that the robot is inside the collection zone?
  return count_left_collection_zone_tags + count_right_collection_zone_tags > 0;

}

//obstacle controller should inrerupt is based upon the transition from not seeing and obstacle to seeing an obstacle
bool ObstacleController::ShouldInterrupt() {

  //if we see and obstacle and havent thrown an interrupt yet
  if(obstacleDetected && !obstacleInterrupt)
  {
    obstacleInterrupt = true;
   //cout<<"SwitchStatus: obstacle controller should interrupt... true"<<endl;
    //cout<<"SwitchStatus: result_cpfa_status="<<result.cpfa_state<<endl;
    return true;
  }
  else
  {
    //if the obstacle has been avoided and we had previously detected one interrupt to change to waypoints
    if(obstacleAvoided && obstacleDetected)
    {
      Reset();
      //cout<<"TestStatus: obstacle interrupt: GetCPFAState()="<<GetCPFAState()<<endl;
      if(GetCPFAState() == return_to_nest)
      {
		  //cout<<"TestStatus: Obstacle avoid and set to reach nest... interrupt...true"<<endl;
		  SetCPFAState(reached_nest);
		  }
      
      
    return true;
    } else {
     //cout<<"false"<<endl;
    return false;
    }
  }
}

bool ObstacleController::HasWork() {
	cout<<"Obstacle has work..."<<endl;
  if (can_set_waypoint && set_waypoint)
  {
    return true;
  }

  return !obstacleAvoided;
}

//ignore center ultrasound
void ObstacleController::SetIgnoreCenterSonar(){
  ignore_center_sonar = true; 
}

void ObstacleController::SetCurrentTimeInMilliSecs( long int time )
{
  current_time = time;
}

void ObstacleController::SetTargetHeld() {
  targetHeld = true;

  //adjust current state on transition from no cube held to cube held
  if (previousTargetState == false) {
    obstacleAvoided = true;
    obstacleInterrupt = false;
    obstacleDetected = false;
    previousTargetState = true;
  }
}

void ObstacleController::SetTargetHeldClear()
{
  //adjust current state on transition from cube held to cube not held
  if (targetHeld)
  {
    Reset();
    targetHeld = false;
    previousTargetState = false;
    ignore_center_sonar = false;
  }
}
CPFAState ObstacleController::GetCPFAState() 
{
  return cpfa_state;
}

void ObstacleController::SetCPFAState(CPFAState state) {
  cpfa_state = state;
  result.cpfa_state = state;
 //cout<<"Obstacle set state: result.cpfa_state ="<<result.cpfa_state <<endl;
}


