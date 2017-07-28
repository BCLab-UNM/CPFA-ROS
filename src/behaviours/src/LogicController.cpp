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
  Result result;

  //first a loop runs through all the controllers who have a priority of 0 or above witht he largest number being
  //most important. A priority of less than 0 is an ignored controller use -1 for standards sake.
  //if any controller needs and interrupt the logic state is changed to interrupt
  for(PrioritizedController cntrlr : prioritizedControllers) {
    if(cntrlr.controller->ShouldInterrupt() && cntrlr.priority >= 0) {
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

    //check what controllers have work to do all that say yes will be added to the priority queue.
    for(PrioritizedController cntrlr : prioritizedControllers) {
      if(cntrlr.controller->HasWork()) {
        if (cntrlr.priority < 0) {
          continue;
        }
        else {
          control_queue.push(cntrlr);
        }
      }
    }

    //if no controlers have work report this to ROS Adapter and do nothing.
    if(control_queue.empty()) {
      result.type = behavior;
      result.b = wait;
      break;
    }
    else {
      //default result state if someone has work this safe gaurds against faulty result types
      result.b = noChange;
    }

    //take the top member of the priority queue and run their do work function.
    result = control_queue.top().controller->DoWork();

    // Check if CPFA state has changed
    //SetCPFAState(result.cpfa_state, result.cpfa_search_type);

    //anaylyze the result that was returned and do state changes accordingly
    //behavior types are used to indicate behavior changes of some form
    if(result.type == behavior) {

      //ask for an external reset so the state of the controller is preserved untill after it has returned a result and
      //gotten a chance to communicate with other controllers
      if (result.reset) {
        controllerInterconnect(); //allow controller to communicate state data before it is reset
        control_queue.top().controller->Reset();
      }

      //ask for the procces state to change to the next state or loop around to the begining
      if(result.b == nextProcess) {
        if (processState == _LAST - 1) {
          processState = _FIRST;
        }
        else {
          processState = (ProcessState)((int)processState + 1);
        }
      }
      //ask for the procces state to change to the previouse state or loop around to the begining
      else if(result.b == prevProcess) {
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

    //ask drive controller how to drive
    //commands to be passed the ROS Adapter as left and right wheel PWM values in the result struct are returned
    result = driveController.DoWork();

    //when out of waypoints drive controller will through an interrupt however unlike other controllers
    //drive controller is not on the priority queue so it must be checked here
    if (result.type == behavior) {
      if(driveController.ShouldInterrupt()) {
        logicState = LOGIC_STATE_INTERRUPT;
      }
    }
    break;
  }//end of waiting case*****************************************************************************************

    //used for precision driving pass through
  case LOGIC_STATE_PRECISION_COMMAND: {

    //unlike waypoints precision commands change every update tick so we ask the
    //controller for new commands on every update tick.
    result = control_queue.top().controller->DoWork();

    //pass the driving commands to the drive controller so it can interpret them
    driveController.SetResultData(result);

    //the interoreted commands are turned into proper motor commands to be passed the ROS Adapter
    //as left and right wheel PWM values in the result struct.
    result = driveController.DoWork();
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

void LogicController::insertPheromone(const std::vector<Point> &pheromone_trail) {
  searchController.insertPheromone(pheromone_trail);
}

void LogicController::SetCPFAState(CPFAState state) {
  if(state != cpfa_state) {
    
    if(state == set_target_location)
      std::cout << "set_target_location" << std::endl;
    else if(state == travel_to_search_site)
      std::cout << "travel_to_search_site" << std::endl;
    else if(state == search_with_uninformed_walk)
      std::cout << "search_with_uninformed_walk" << std::endl;
    else if(state == search_with_informed_walk)
      std::cout << "search_with_informed_walk" << std::endl;
    else if(state == sense_local_resource_density)
      std::cout << "sense_local_resource_density" << std::endl;
    else
      std::cout << "return_to_nest" << std::endl;

    cpfa_state = state;
    for(PrioritizedController cntrlr : prioritizedControllers) {
      if(state != cntrlr.controller->GetCPFAState()) {
        cntrlr.controller->SetCPFAState(state);
      }
    }
  }
}

CPFAState LogicController::GetCPFAState() {
  return cpfa_state;
}

void LogicController::SetCPFASearchType(CPFASearchType search_type) {

  if(search_type != cpfa_search_type) {

    if(search_type == site_fidelity)
      std::cout << "cpfa_search_type site_fidelity" << std::endl;
    else if(search_type == pheromone)
      std::cout << "cpfa_search_type pheromone" << std::endl;
    else
      std::cout << "cpfa_search_type random_search" << std::endl;

    cpfa_search_type = search_type;
    for(PrioritizedController cntrlr : prioritizedControllers) {
      if(search_type != cntrlr.controller->GetCPFASearchType()) {
        cntrlr.controller->SetCPFASearchType(search_type);
      }
    }

  }

}

CPFASearchType LogicController::GetCPFASearchType() {
  return cpfa_search_type;
}
