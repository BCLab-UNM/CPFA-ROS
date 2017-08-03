#include "LogicController.h"

LogicController::LogicController() {
  logicState = LOGIC_STATE_INTERRUPT;
  processState = PROCCESS_STATE_SEARCHING;

  ProcessData();

  control_queue = priority_queue<PrioritizedController>();

}

LogicController::~LogicController() {}

void LogicController::Reset() {

  logicState = LOGIC_STATE_INTERRUPT;
  processState = PROCCESS_STATE_SEARCHING;

  ProcessData();

  control_queue = priority_queue<PrioritizedController>();
}

//***********************************************************************************************************************
//This function is called every 1/10th second by the ROSAdapter
//The logical flow if the behaviours is controlled here by using a interrupt, haswork, priority queue system.
Result LogicController::DoWork() {
  cout << "LogicController doing work..." << endl;
  Result result;

  //first a loop runs through all the controllers who have a priority of 0 or above witht he largest number being
  //most important. A priority of less than 0 is an ignored controller use -1 for standards sake.
  //if any controller needs and interrupt the logic state is changed to interrupt
  cout << "Checking to see if controllers should interrupt..." << endl;
  for(PrioritizedController cntrlr : prioritizedControllers) {
    if(cntrlr.controller->ShouldInterrupt() && cntrlr.priority >= 0) {
      cout << cntrlr.controller->name << " interrupting!" << endl;
      logicState = LOGIC_STATE_INTERRUPT;
      //do not break all shouldInterupts may need calling in order to properly pre-proccess data.
    }
  }

  //logic state switch
  switch(logicState) {

  //when an interrupt has been thorwn or there are no pending actions logic controller is in this state.
  case LOGIC_STATE_INTERRUPT: {

    //Reset the control queue
    control_queue = priority_queue<PrioritizedController>();
    cout << endl << "logicstate: LOGIC_STATE_INTERRUPT" << endl;

    //check what controllers have work to do all that say yes will be added to the priority queue.
    cout << "Checking to see if controllers have work..." << endl;
    for(PrioritizedController cntrlr : prioritizedControllers) {
      if(cntrlr.controller->HasWork()) {
        if (cntrlr.priority < 0) {
          continue;
        }
        else {
          cout << cntrlr.controller->name << " has work!" << endl;
          control_queue.push(cntrlr);
        }
      }
    }

    //if no controlers have work report this to ROS Adapter and do nothing.
    if(control_queue.empty()) {
      cout << "control queue is empty" << endl;
      result.type = behavior;
      result.b = wait;
      break;
    }
    else {
      //default result state if someone has work this safe gaurds against faulty result types
      result.b = noChange;
    }

    //take the top member of the priority queue and run their do work function.
    cout << endl << control_queue.top().controller->name << ": doing work!" << endl;
    result = control_queue.top().controller->DoWork();
    cout << "obstacleController turn_direction: " << obstacleController.getTurnDirection() << endl;

    // This tells ROS Adapter whether to lay a pheromone
    lay_pheromone = result.lay_pheromone;

    // Check if CPFA state has changed
    SetCPFAState(result.cpfa_state);
    SetCPFASearchType(result.cpfa_search_type);

    //anaylyze the result that was returned and do state changes accordingly
    //behavior types are used to indicate behavior changes of some form
    if(result.type == behavior) {
      cout << "result.type == behavior" << endl;

      //ask for an external reset so the state of the controller is preserved untill after it has returned a result and
      //gotten a chance to communicate with other controllers
      if (result.reset) {
        cout << "result.reset" << endl;
        controllerInterconnect(); //allow controller to communicate state data before it is reset
        control_queue.top().controller->Reset();
      }

      //ask for the procces state to change to the next state or loop around to the begining
      if(result.b == nextProcess) {
        cout << "result.b == nextProcess" << endl;
        if (processState == _LAST - 1) {
          processState = _FIRST;
        }
        else {
          processState = (ProcessState)((int)processState + 1);
        }
      }
      //ask for the procces state to change to the previouse state or loop around to the begining
      else if(result.b == prevProcess) {
        cout << "result.b == prevProcess" << endl;
        if (processState == _FIRST) {
          processState = (ProcessState)((int)_LAST - 1);
        }
        else {
          processState = (ProcessState)((int)processState - 1);
        }
      }

      //update the priorites of the controllers based upon the new process state.
      if (result.b == nextProcess || result.b == prevProcess) {
        ProcessData();
        result.b = wait;
        driveController.Reset(); //it is assumed that the drive controller may be in a bad state if interrupted so reset it
        cout << "Resetting driveController..." << endl;
      }

      break;
    }

    //precision driving result types are when a controller wants direct command of the robots actuators
    //logic controller facilitates the command pass through in the LOGIC_STATE_PRECISION_COMMAND switch case
    else if(result.type == precisionDriving) {

      logicState = LOGIC_STATE_PRECISION_COMMAND;
      break;

    }

    //waypoints are also a pass through facilitated command but with a slightly diffrent overhead
    //they are handled in the LOGIC_STATE_WAITING switch case
    else if(result.type == waypoint) {

      logicState = LOGIC_STATE_WAITING;
      driveController.SetResultData(result);
      //fall through on purpose
    }

  } //end of interupt case***************************************************************************************

    //this case is primarly when logic controller is waiting for drive controller to reach its last waypoint
  case LOGIC_STATE_WAITING: {

    cout << endl << "logicState: LOGIC_STATE_WAITING" << endl;
    //ask drive controller how to drive
    //commands to be passed the ROS Adapter as left and right wheel PWM values in the result struct are returned
    cout << "DriveController doing work..." << endl;
    result = driveController.DoWork();

    // Check if CPFA state has changed
    SetCPFAState(result.cpfa_state);
    SetCPFASearchType(result.cpfa_search_type);
    cout << endl;

    //when out of waypoints drive controller will through an interrupt however unlike other controllers
    //drive controller is not on the priority queue so it must be checked here
    if (result.type == behavior) {
      if(driveController.ShouldInterrupt()) {
        cout << "DriveController interrupting..." << endl;
        logicState = LOGIC_STATE_INTERRUPT;
      }
    }
    break;
  }//end of waiting case*****************************************************************************************

    //used for precision driving pass through
  case LOGIC_STATE_PRECISION_COMMAND: {
    cout << endl << "logicState: LOGIC_STATE_PRECISION_COMMAND" << endl;
    //unlike waypoints precision commands change every update tick so we ask the
    //controller for new commands on every update tick.
    cout << control_queue.top().controller->name << ": doing work!" << endl;
    result = control_queue.top().controller->DoWork();

    // Check if CPFA state has changed
    SetCPFAState(result.cpfa_state);
    SetCPFASearchType(result.cpfa_search_type);

    //pass the driving commands to the drive controller so it can interpret them
    cout << endl << control_queue.top().controller->name << ": doing work!" << endl;
    driveController.SetResultData(result);

    //the interoreted commands are turned into proper motor commands to be passed the ROS Adapter
    //as left and right wheel PWM values in the result struct.
    result = driveController.DoWork();

    // Check if CPFA state has changed
    SetCPFAState(result.cpfa_state);
    SetCPFASearchType(result.cpfa_search_type);

    break;

  }//end of precision case****************************************************************************************
  }//end switch statment******************************************************************************************


  //now using proccess logic allow the controller to communicate data between eachother
  controllerInterconnect();

  //give the ROSAdapter the final decision on how it should drive
  return result;
}

void LogicController::UpdateData() {


}

void LogicController::ProcessData() {

  //this controller priority is used when searching
  if (processState == PROCCESS_STATE_SEARCHING) {
    prioritizedControllers = {
      PrioritizedController{0, (Controller*)(&searchController)},
      PrioritizedController{1, (Controller*)(&obstacleController)},
      PrioritizedController{2, (Controller*)(&pickUpController)},
      PrioritizedController{-1, (Controller*)(&dropOffController)}
    };
  }

  //this priority is used when returning a target to the center collection zone
  else if (processState  == PROCCESS_STATE_TARGET_PICKEDUP) {
    prioritizedControllers = {
      PrioritizedController{-1, (Controller*)(&searchController)},
      PrioritizedController{2, (Controller*)(&obstacleController)},
      PrioritizedController{-1, (Controller*)(&pickUpController)},
      PrioritizedController{1, (Controller*)(&dropOffController)}
    };
  }
}

bool LogicController::ShouldInterrupt() {
  ProcessData();

  return false;
}

bool LogicController::HasWork() {
  return false;
}


void LogicController::controllerInterconnect() {

  if (processState == PROCCESS_STATE_SEARCHING) {

    //obstacle needs to know if the center ultrasound should be ignored
    if(pickUpController.GetIgnoreCenter()) {
      obstacleController.SetIgnoreCenter();
    }

    //pickup controller annouces it has pickedup a target
    if(pickUpController.GetTargetHeld()) {
      dropOffController.SetTargetPickedUp();
      obstacleController.SetTargetHeld();
      searchController.SetSuccesfullPickup();
    }
  }

  //ask if drop off has released the target from the claws yet
  if (!dropOffController.HasTarget()) {
    obstacleController.SetTargetHeldClear();
  }

  //obstacle controller is running driveController needs to clear its waypoints
  if(obstacleController.GetShouldClearWaypoints()) {
    driveController.Reset();
  }

  // Allow search controller to insert waypoint to avoid obstacle
  if(obstacleController.HasWork())
  {
    searchController.setObstacleAvoidance(obstacleController.getTurnDirection());
  }

}


void LogicController::SetPositionData(Point currentLocation) {
  searchController.SetCurrentLocation(currentLocation);
  dropOffController.SetCurrentLocation(currentLocation);
  obstacleController.SetCurrentLocation(currentLocation);
  driveController.SetCurrentLocation(currentLocation);
}

void LogicController::SetMapPositionData(Point currentLocationMap) {

}

void LogicController::SetVelocityData(float linearVelocity, float angularVelocity) {
  driveController.SetVelocityData(linearVelocity,angularVelocity);
}

void LogicController::SetMapVelocityData(float linearVelocity, float angularVelocity) {

}

void LogicController::SetAprilTags(vector<TagPoint> tags) {
  // If giving up and returning to nest
  if(!pickUpController.GetTargetHeld() && cpfa_state == return_to_nest)
  {
    int size = tags.size();
    for(int i = 0; i < size; i++) 
    {
      if(tags[i].id == 256)
      {
        SetCPFAState(set_target_location);
        break;
      }
    }
  }

  pickUpController.SetTagData(tags);
  obstacleController.SetTagData(tags);
  dropOffController.SetTargetData(tags);
}

void LogicController::SetSonarData(float left, float center, float right) {
  pickUpController.SetSonarData(center);
  obstacleController.SetSonarData(left,center,right);
}

void LogicController::SetCenterLocationOdom(Point centerLocationOdom) {
  searchController.SetCenterLocation(centerLocationOdom);
  dropOffController.SetCenterLocation(centerLocationOdom);
}

void LogicController::SetCenterLocationMap(Point centerLocationMap) {

}

void LogicController::SetCurrentTimeInMilliSecs( long int time )
{
  current_time = time;
  dropOffController.SetCurrentTimeInMilliSecs( time );
  pickUpController.SetCurrentTimeInMilliSecs( time );
  obstacleController.SetCurrentTimeInMilliSecs( time );
}

void LogicController::SetArenaSize(int numRovers) {
  searchController.setArenaSize(numRovers);
}

void LogicController::insertPheromone(const vector<Point> &pheromone_trail) {
  searchController.insertPheromone(pheromone_trail);
}

void LogicController::SetCPFAState(CPFAState state) {
  if(state != cpfa_state) {
    cpfa_state = state;
    for(PrioritizedController cntrlr : prioritizedControllers) {
      if(state != cntrlr.controller->GetCPFAState()) {
        cntrlr.controller->SetCPFAState(state);
      }
    }
  }

  printCPFAState();
}

CPFAState LogicController::GetCPFAState() {
  return cpfa_state;
}

void LogicController::SetCPFASearchType(CPFASearchType search_type) {
  if(search_type != cpfa_search_type) {
    cpfa_search_type = search_type;
    for(PrioritizedController cntrlr : prioritizedControllers) {
      if(search_type != cntrlr.controller->GetCPFASearchType()) {
        cntrlr.controller->SetCPFASearchType(search_type);
      }
    }

  }

  printCPFASearchType();
}

CPFASearchType LogicController::GetCPFASearchType() {
  return cpfa_search_type;
}

void LogicController::senseLocalResourceDensity(int num_tags) {
  if(cpfa_state == sense_local_resource_density) {
    searchController.senseLocalResourceDensity(num_tags);

    cout << "CPFAState: sense_local_resource_density" << endl;
    printCPFASearchType();
  }
}

void LogicController::printCPFAState() {
  cout << "CPFAState: ";
  if(cpfa_state == set_target_location)
    cout << "set_target_location" << endl;
  else if(cpfa_state == travel_to_search_site)
    cout << "travel_to_search_site" << endl;
  else if(cpfa_state == search_with_uninformed_walk)
    cout << "search_with_uninformed_walk" << endl;
  else if(cpfa_state == search_with_informed_walk)
    cout << "search_with_informed_walk" << endl;
  else if(cpfa_state == sense_local_resource_density)
    cout << "sense_local_resource_density" << endl;
  else if (cpfa_state == return_to_nest)
    cout << "return_to_nest" << endl;
  else
    cout << "WTF" << endl;
}

void LogicController::printCPFASearchType() {
    cout << "CPFASearchType: ";
    if(cpfa_search_type == site_fidelity)
      cout << "site_fidelity" << endl;
    else if(cpfa_search_type == pheromone)
      cout << "pheromone" << endl;
    else if(cpfa_search_type == random_search)
      cout << "random_search" << endl;
    else 
      cout << "WTF" << endl;
}

bool LogicController::layPheromone() {

  if(lay_pheromone) {
    lay_pheromone = false;
    return searchController.layPheromone();
  }

  return false;
}

Point LogicController::getTargetLocation() {
  return searchController.getTargetLocation();
}
