#include "LogicController.h"

using namespace std;

LogicController::LogicController() {
  logicState = LOGIC_STATE_INTERRUPT;
  processState = PROCESS_STATE_SEARCHING;

  // Melanie wants fixed seeds
  srand(CPFA_parameters.random_seed);

  ProcessData();

  control_queue = priority_queue<PrioritizedController>();

}

LogicController::~LogicController() {}

void LogicController::Reset() {

  logicState = LOGIC_STATE_INTERRUPT;
  processState = PROCESS_STATE_SEARCHING;

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

  //when an interrupt has been thorwn or there are no pending control_queue.top().actions logic controller is in this state.
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

    //anaylyze the result that was returned and do state changes accordingly
    //behavior types are used to indicate behavior changes of some form
    if(result.type == behavior) {

      //ask for an external reset so the state of the controller is preserved untill after it has returned a result and
      //gotten a chance to communicate with other controllers
      if (result.reset) {
        controllerInterconnect(); //allow controller to communicate state data before it is reset
        control_queue.top().controller->Reset();
      }

      //update the priorites of the controllers based upon the new process state.
      if (result.b == COMPLETED || result.b == FAILED) {
        
        // Custom code starts here
        // what do you want to do here?
        processState = CPFAStateMachine(processState);
        // Custom code ends here

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

    //the interoreted commands are turned into properinitial_spiral_offset motor commands to be passed the ROS Adapter
    //as left and right wheel PWM values in the result struct.
    result = driveController.DoWork();
    break;

  }//end of precision case****************************************************************************************
  }//end switch statment******************************************************************************************

    cout << "logic state " << logicState << " top controller " << control_queue.top().priority << " Proccess " << processState <<endl;


  //now using proccess logic allow the controller to communicate data between eachother
  controllerInterconnect();

  //give the ROSAdapter the final decision on how it should drive
  return result;
}

void LogicController::UpdateData() {


}

void LogicController::ProcessData() {

  //this controller priority is used when searching
  if (processState == PROCESS_STATE_START)
  {
    prioritizedControllers = {
      PrioritizedController{0, (Controller*)(&searchController)},
      PrioritizedController{1, (Controller*)(&obstacleController)},
      PrioritizedController{-1, (Controller*)(&dropOffController)},
      PrioritizedController{-1, (Controller*)(&pickUpController)},
    };
  }
  else if (processState == PROCESS_STATE_SITE_FIDELITY)
  {
    prioritizedControllers = {
      PrioritizedController{1, (Controller*)(&obstacleController)},
      PrioritizedController{0, (Controller*)(&site_fidelity_controller)},
      PrioritizedController{-1, (Controller*)(&searchController)},
      PrioritizedController{-1, (Controller*)(&pickUpController)},
      PrioritizedController{-1, (Controller*)(&dropOffController)}
    };
  }
  else if (processState == PROCESS_STATE_PHEROMONE)
  {
    prioritizedControllers = {
      PrioritizedController{1, (Controller*)(&obstacleController)},
      //PrioritizedController{0, (Controller*)(&pheromone_controller)},
      PrioritizedController{-1, (Controller*)(&searchController)},
      PrioritizedController{-1, (Controller*)(&pickUpController)},
      PrioritizedController{-1, (Controller*)(&dropOffController)}
    };
  }
  else if (processState == PROCESS_STATE_RANDOM_DISPERSAL)
  {
    prioritizedControllers = {
      PrioritizedController{1, (Controller*)(&obstacleController)},
      //PrioritizedController{0, (Controller*)(&random_dispersal_controller)},
      PrioritizedController{-1, (Controller*)(&searchController)},
      PrioritizedController{-1, (Controller*)(&pickUpController)},
      PrioritizedController{-1, (Controller*)(&dropOffController)}
    };
  }
  else if (processState == PROCESS_STATE_SEARCHING)
  {
    prioritizedControllers = {
      PrioritizedController{2, (Controller*)(&pickUpController)},
      PrioritizedController{1, (Controller*)(&obstacleController)},
      PrioritizedController{0, (Controller*)(&searchController)},
      PrioritizedController{-1, (Controller*)(&dropOffController)}
    };
  }

  //this priority is used when returning a target to the center collection zone
  else if (processState  == PROCESS_STATE_RETURN_TO_NEST) {
    prioritizedControllers = {
      PrioritizedController{1, (Controller*)(&obstacleController)},
      //PrioritizedController{0, (Controller*)(&return_to_nest_controller)},
      PrioritizedController{-1, (Controller*)(&searchController)},
      PrioritizedController{-1, (Controller*)(&pickUpController)},
      PrioritizedController{-1, (Controller*)(&dropOffController)}
    };
  }
  //this priority is used when returning a target to the center collection zone
  else if (processState  == PROCESS_STATE_DROPOFF)
  {
    prioritizedControllers = {
      PrioritizedController{-1, (Controller*)(&searchController)},
      PrioritizedController{1, (Controller*)(&obstacleController)},
      PrioritizedController{-1, (Controller*)(&pickUpController)},
      PrioritizedController{0, (Controller*)(&dropOffController)}
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

  if (processState == PROCESS_STATE_SEARCHING) {

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
  site_fidelity_controller.setCurrentLocation(currentLocation);
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

ProcessState LogicController::CPFAStateMachine(ProcessState state) 
{
  ProcessState new_state;

  // Determine next process state for appropriate CPFA state
  switch(state) {
    case PROCESS_STATE_START:
      new_state = PROCESS_STATE_RANDOM_DISPERSAL;
      break;

      // Sets target location to a site fidelity location, pheromone, or random location.
      // Preset when rover has picked up a resource and is returning to collection zone
    case PROCESS_STATE_SITE_FIDELITY:
    case PROCESS_STATE_PHEROMONE:
    case PROCESS_STATE_RANDOM_DISPERSAL:
      new_state = PROCESS_STATE_SEARCHING;
      break;

      // Searches at a site fidelity location or pheromone location
    case PROCESS_STATE_SEARCHING:
      new_state = PROCESS_STATE_RETURN_TO_NEST;
      break;

      // Counts local resource density while picking up a resource
    case PROCESS_STATE_RETURN_TO_NEST:
      if (target_held) 
      {
        new_state = PROCESS_STATE_DROPOFF;
      }
      else if (num_pheromones > 0)
      {
        new_state = PROCESS_STATE_PHEROMONE;
      }
      else
      {
        new_state = PROCESS_STATE_RANDOM_DISPERSAL;
      }
      break;

      // Only set when Rover has given up search
    case PROCESS_STATE_DROPOFF:
      if (PoissonCDF(CPFA_parameters.rate_of_site_fidelity) > rand() * 1.0f/INT_MAX)
      {
        new_state = PROCESS_STATE_SITE_FIDELITY;
      }
      else if (num_pheromones > 0)
      {
        new_state = PROCESS_STATE_PHEROMONE;
      }
      else 
      {
        new_state = PROCESS_STATE_RANDOM_DISPERSAL;
      }
      break;
  }
  return new_state;
}


double LogicController::PoissonCDF(double lambda)
{
  double sumAccumulator       = 1.0;
  double factorialAccumulator = 1.0;

  for (size_t i = 1; i <= local_resource_density; i++) {
    factorialAccumulator *= i;
    sumAccumulator += pow(lambda, i) / factorialAccumulator;
  }

  return (exp(-lambda) * sumAccumulator);
}
