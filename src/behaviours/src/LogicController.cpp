#include "LogicController.h"

using namespace std;

LogicController::LogicController() {
  cout<<"start logic controller..."<<endl;
  logicState = LOGIC_STATE_INTERRUPT;
  processState = PROCCESS_STATE_SEARCHING;
  rng = new random_numbers::RandomNumberGenerator();
  // Melanie wants fixed seeds
  //srand(CPFA_parameters.random_seed);
  
  ProcessData();
  
  control_queue = priority_queue<PrioritizedController>();
  
}

LogicController::~LogicController() {}

void LogicController::Reset() {
  std::cout << "LogicController.Reset()" << std::endl;
  logicState = LOGIC_STATE_INTERRUPT;
  processState = PROCCESS_STATE_SEARCHING;
  cout<<"logicState="<<logicState<<endl;
  cout<<"processState="<<processState<<endl;
  ProcessData();
  

  control_queue = priority_queue<PrioritizedController>();
}
	
//***********************************************************************************************************************
//This function is called every 1/10th second by the ROSAdapter
//The logical flow if the behaviours is controlled here by using a interrupt, haswork, priority queue system.
Result LogicController::DoWork() {
  Result result;
  
  cout << "LogicController:DoWork()...Proccess State is " << processState << endl;
  //first a loop runs through all the controllers who have a priority of 0 or above with the largest number being
  //most important. A priority of less than 0 is an ignored controller use -1 for standards sake.
  //if any controller needs and interrupt the logic state is changed to interrupt
  for(PrioritizedController cntrlr : prioritizedControllers) {
    if(cntrlr.controller->ShouldInterrupt() && cntrlr.priority >= 0)
    {
		cout<<" is interrupt..."<<endl;
      logicState = LOGIC_STATE_INTERRUPT;
      //do not break all shouldInterupts may need calling in order to properly pre-proccess data.
    }
  }
  
  cout <<"logicState="<<logicState<<endl;
  //logic state switch
  switch(logicState) {
  
  //when an interrupt has been thorwn or there are no pending control_queue.top().actions logic controller is in this state.
  case LOGIC_STATE_INTERRUPT: {
    //Reset the control queue
    control_queue = priority_queue<PrioritizedController>();
    
    //check what controllers have work to do all that say yes will be added to the priority queue.
    for(PrioritizedController cntrlr : prioritizedControllers) {
		cout <<"priority="<<cntrlr.priority<<endl;
		cout<< "name="<<cntrlr.controller <<endl;
      if(cntrlr.controller->HasWork()) {
        if (cntrlr.priority < 0) {
			cout<<"continue ... "<<endl;
          continue;
        }
        else {
          control_queue.push(cntrlr);
          cout<<"cntrlr..."<<endl;
        }
      }
    }
    //if no controlers have work report this to ROS Adapter and do nothing.
    if(control_queue.empty()) {
		cout<<"control_queue.empty()"<<endl;
      result.type = behavior;
      result.b = wait;
      break;
    }
    else {
      //default result state if someone has work this safe guards against faulty result types
      result.b = noChange;
      cout<<"behavior is no change"<<endl;
    }
    
    //take the top member of the priority queue and run their do work function.
    result = control_queue.top().controller->DoWork();
    
    cout<<"get the result..."<<endl;
    //anaylyze the result that was returned and do state changes accordingly
    //behavior types are used to indicate behavior changes of some form
    cout<<"result.type="<<result.type<<endl;
    if(result.type == behavior) {
      cout<<"result type == behavior"<<endl;
      //ask for an external reset so the state of the controller is preserved untill after it has returned a result and
      //gotten a chance to communicate with other controllers
      if (result.reset) {
		  cout<<"result reset..."<<endl;
        controllerInterconnect(); //allow controller to communicate state data before it is reset
        cout<<"controller="<<control_queue.top().controller<<endl;
        control_queue.top().controller->Reset();
      }
      //ask for the procces state to change to the next state or loop around to the begining
      if(result.b == nextProcess) {
		  cout<<"next process..."<<endl;
        if (processState == _LAST - 1) {
          processState = _FIRST;
        }
        else {
          processState = (ProcessState)((int)processState + 1);
        }
      }
      //ask for the procces state to change to the previouse state or loop around to the end
      else if(result.b == prevProcess) {
        cout<<"previous process..."<<endl;
        if (processState == _FIRST) {
          processState = (ProcessState)((int)_LAST - 1);
        }
        else {
          processState = (ProcessState)((int)processState - 1);
        }
      }
      //update the priorites of the controllers based upon the new process state.
      if (result.b == nextProcess || result.b == prevProcess) 
      {
		  cout<<"result.b == next or prev"<<endl;
		  if(processState == PROCESS_STATE_SITE_FIDELITY)
		  {
		       double followSiteFidelityRate = getPoissonCDF(rate_of_laying_pheromone);
               cout <<"CPFAStatus: followSiteFidelityRate="<<followSiteFidelityRate<<endl;
               double r1 = rng->uniformReal(0, 1);
               cout<<"CPFAStatus: r1 = "<< r1<<endl;
               if(r1 > followSiteFidelityRate)//informed search with pheromone waypoints
               {
		           //result.type = behavior;
                   //result.b = nextProcess;		    
                   processState = (ProcessState)((int)processState + 1); 
                   cout <<"CPFAStatus: processState ="<<processState<<"  pheromome"<<endl;
               }
           }
	     
         ProcessData();
         result.b = wait;
         driveController.Reset(); //it is assumed that the drive controller may be in a bad state if interrupted so reset it
      }
      else if(result.b == COMPLETED || result.b == FAILED){
		  cout <<"CPFAStatus: Completed..."<<endl;
		  informed_search = true;
          processState = PROCCESS_STATE_SEARCHING;
          ProcessData();
          result.b = wait;
          driveController.Reset();
      }
      break;
    }

    //precision driving result types are when a controller wants direct command of the robots actuatorshandler
    //logic controller facilitates the command pass through in the LOGIC_STATE_PRECISION_COMMAND switch case
    else if(result.type == precisionDriving) {
      cout<<"result type == precisionDriving..."<<endl;
      logicState = LOGIC_STATE_PRECISION_COMMAND;
      break;

    }

    //waypoints are also a pass through facilitated command but with a slightly diffrent overhead
    //they are handled in the LOGIC_STATE_WAITING switch case
    else if(result.type == waypoint) {
      cout<<"result type == waypoint"<<endl;
      logicState = LOGIC_STATE_WAITING;
      cout<<"logicState = waiting/1"<<endl;
      driveController.SetResultData(result);
      //fall through on purpose
    }

  } //end of interupt case***************************************************************************************

    //this case is primarly when logic controller is waiting for drive controller to reach its last waypoint
  case LOGIC_STATE_WAITING: {
     cout <<"logic state waiting..."<<endl;
    //ask drive controller how to drive
    //commands to be passed the ROS Adapter as left and right wheel PWM values in the result struct are returned
    result = driveController.DoWork();
    
    //when out of waypoints drive controller will through an interrupt however unlike other controllers
    //drive controller is not on the priority queue so it must be checked here
    if (result.type == behavior) {
		cout <<"logic state== waiting; result type == behavior"<<endl;
      if(driveController.ShouldInterrupt()) {
        logicState = LOGIC_STATE_INTERRUPT;
        cout<<"driver controller interrupt..."<<endl;
      }
    }
    break;
  }//end of waiting case*****************************************************************************************

    //used for precision driving pass through
  case LOGIC_STATE_PRECISION_COMMAND: {
    cout<<"logic state precision command..."<<endl;
    //unlike waypoints precision commands change every update tick so we ask the
    //controller for new commands on every update tick.
    cout<<"control_queue.top().controller="<<control_queue.top().controller<<endl;
    result = control_queue.top().controller->DoWork();

    //pass the driving commands to the drive controller so it can interpret them
    driveController.SetResultData(result);

    //the interoreted commands are turned into properinitial_spiral_offset motor commands to be passed the ROS Adapter
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

void LogicController::UpdateData() 
{


}

double LogicController::getPoissonCDF(const double lambda)
{
  double sumAccumulator       = 1.0;
  double factorialAccumulator = 1.0;
   cout <<"CPFAStatus: lambda="<<lambda<<endl;
   local_resource_density =8;//should be removed. Just for testing
   cout <<"CPFAStatus: get Poisson CDF: local_resource_density="<<local_resource_density<<endl;
  for (size_t i = 1; i <= local_resource_density; i++) {
    factorialAccumulator *= i;
    sumAccumulator += pow(lambda, i) / factorialAccumulator;
  }

  return (exp(-lambda) * sumAccumulator);
}


void LogicController::ProcessData() {
  cout<<"CPFAStatus: ";
  //this controller priority is used when searching
  if (processState == PROCCESS_STATE_SEARCHING) 
  {
	  cout<<"searching ..."<<endl;
	prioritizedControllers = {
	PrioritizedController{15, (Controller*)(&pickUpController)},
	PrioritizedController{10, (Controller*)(&obstacleController)},
	PrioritizedController{5, (Controller*)(&rangeController)},
	PrioritizedController{0, (Controller*)(&searchController)},
	PrioritizedController{-1, (Controller*)(&dropOffController)},
	PrioritizedController{-1, (Controller*)(&manualWaypointController)}
    };
  }
  else if (processState == PROCESS_STATE_SITE_FIDELITY)
  {
  	  cout<<"SF: site fidelity ..."<<endl;
    prioritizedControllers = {
      PrioritizedController{1, (Controller*)(&obstacleController)},
      PrioritizedController{0, (Controller*)(&siteFidelityController)},
      PrioritizedController{-1, (Controller*)(&searchController)},
      PrioritizedController{-1, (Controller*)(&pickUpController)},
      PrioritizedController{-1, (Controller*)(&dropOffController)}
    };
  }
  else if (processState == PROCESS_STATE_PHEROMONE)
  {
     cout<<"pheromone ..."<<endl;
    prioritizedControllers = {
      PrioritizedController{1, (Controller*)(&obstacleController)},
      PrioritizedController{0, (Controller*)(&pheromoneController)},
      PrioritizedController{-1, (Controller*)(&searchController)},
      PrioritizedController{-1, (Controller*)(&pickUpController)},
      PrioritizedController{-1, (Controller*)(&dropOffController)}
    };
  }
  //this priority is used when returning a target to the center collection zone
  else if (processState  == PROCCESS_STATE_TARGET_PICKEDUP) 
  {
	  cout<<"target pick up ..."<<endl;
    prioritizedControllers = {
    PrioritizedController{15, (Controller*)(&obstacleController)},
    PrioritizedController{10, (Controller*)(&rangeController)},
    PrioritizedController{1, (Controller*)(&dropOffController)},
    PrioritizedController{-1, (Controller*)(&searchController)},
    PrioritizedController{-1, (Controller*)(&pickUpController)},
    PrioritizedController{-1, (Controller*)(&manualWaypointController)}
    };
  }
  //this priority is used when returning a target to the center collection zone
  else if (processState  == PROCCESS_STATE_DROP_OFF)
  {
	  cout<<"target drop off ..."<<endl;
      prioritizedControllers = {
	  PrioritizedController{10, (Controller*)(&rangeController)},
	  PrioritizedController{1, (Controller*)(&dropOffController)},
      PrioritizedController{-1, (Controller*)(&searchController)},
      PrioritizedController{-1, (Controller*)(&obstacleController)},
      PrioritizedController{-1, (Controller*)(&pickUpController)},
      PrioritizedController{-1, (Controller*)(&manualWaypointController)}
    };
  }
  else if (processState == PROCCESS_STATE_MANUAL) {
    prioritizedControllers = {
      PrioritizedController{5,  (Controller*)(&manualWaypointController)},
      PrioritizedController{-1, (Controller*)(&searchController)},
      PrioritizedController{-1, (Controller*)(&obstacleController)},
      PrioritizedController{-1, (Controller*)(&pickUpController)},
      PrioritizedController{-1, (Controller*)(&rangeController)},
      PrioritizedController{-1, (Controller*)(&dropOffController)}
    };
  }
}

bool LogicController::ShouldInterrupt() 
{
	cout<<"logic controller should not interrupt..."<<endl;
  ProcessData();
  return false;
}

bool LogicController::HasWork()
{
  return false;
}

void LogicController::controllerInterconnect() 
{
  cout<<"controller interconnect..."<<endl;
  cout<<"processState="<<processState<<endl;
  if (processState == PROCCESS_STATE_SEARCHING) 
  {
    cout<<"state searching" <<endl;
    //obstacle needs to know if the center ultrasound should be ignored
    if(pickUpController.GetIgnoreCenter()) 
    {
		cout<<"get ingnore center..."<<endl;
      obstacleController.setIgnoreCenterSonar();
    }

    //pickup controller annouces it has pickedup a target
    if(pickUpController.GetTargetHeld()) 
    {
		cout<<"get target held..."<<endl;
      dropOffController.SetTargetPickedUp();
      obstacleController.setTargetHeld();
      searchController.SetSuccesfullPickup();
      siteFidelityController.setTargetPickedUp();
      //targetHeld = true;
    }
  }

  //ask if drop off has released the target from the claws yet
  if (!dropOffController.HasTarget()) 
  {
	  cout<<"drop off: does not have target..."<<endl;
    obstacleController.setTargetHeldClear();
  }

  //obstacle controller is running driveController needs to clear its waypoints
  if(obstacleController.getShouldClearWaypoints())
  {
	  cout<<"clear waypoints..."<<endl;
    driveController.Reset();
  }
  
  // Let search controller know whether it should use an
  // informed search or an uninformed search
  if (informed_search)
  {
	  cout<<"set search type..."<<endl;
    searchController.setSearchType(informed_search);
    informed_search = false;
  }
  
  /*if (targetHeld)
  {
    informed_search = false;
    searchController.setSearchType(informed_search);
  }*/
  
}
// Recieves position in the world inertial frame (should rename to SetOdomPositionData)
void LogicController::SetPositionData(Point currentLocation) 
{
  searchController.SetCurrentLocation(currentLocation);
  dropOffController.SetCurrentLocation(currentLocation);
  obstacleController.setCurrentLocation(currentLocation);
  driveController.SetCurrentLocation(currentLocation);
  manualWaypointController.SetCurrentLocation(currentLocation);
  siteFidelityController.setCurrentLocation(currentLocation);
  //return_to_nest_controller.SetCurrentLocation(currentLocation);
  randomDispersalController.setCurrentLocation(currentLocation);
}


// Recieves position in the world frame with global data (GPS)
void LogicController::SetMapPositionData(Point currentLocation) 
{
  rangeController.setCurrentLocation(currentLocation);  
}

void LogicController::SetVelocityData(float linearVelocity, float angularVelocity) 
{
  driveController.SetVelocityData(linearVelocity,angularVelocity);
}

void LogicController::SetMapVelocityData(float linearVelocity, float angularVelocity) 
{

}

void LogicController::SetAprilTags(vector<Tag> tags) 
{
  pickUpController.SetTagData(tags);
  obstacleController.setTagData(tags);
  dropOffController.SetTargetData(tags);
  //return_to_nest_controller.SetTargetData(tags);
}

void LogicController::SetSonarData(float left, float center, float right) 
{
  pickUpController.SetSonarData(center);
  obstacleController.setSonarData(left,center,right);
}

// Called once by RosAdapter in guarded init
void LogicController::SetCenterLocationOdom(Point centerLocationOdom) 
{
  searchController.SetCenterLocation(centerLocationOdom);
  dropOffController.SetCenterLocation(centerLocationOdom);
  //return_to_nest_controller.SetCenterLocation(centerLocationOdom);
}

void LogicController::AddManualWaypoint(Point manualWaypoint, int waypoint_id)
{
  manualWaypointController.AddManualWaypoint(manualWaypoint, waypoint_id);
}

void LogicController::RemoveManualWaypoint(int waypoint_id)
{
  manualWaypointController.RemoveManualWaypoint(waypoint_id);
}

std::vector<int> LogicController::GetClearedWaypoints()
{
  return manualWaypointController.ReachedWaypoints();
}

void LogicController::setVirtualFenceOn( RangeShape* range )
{
  rangeController.setRangeShape(range);
  rangeController.setEnabled(true);
}

void LogicController::setVirtualFenceOff()
{
  rangeController.setEnabled(false);
}






void LogicController::SetCenterLocationMap(Point centerLocationMap) 
{

}

void LogicController::SetCurrentTimeInMilliSecs( long int time )
{
  current_time = time;
  dropOffController.SetCurrentTimeInMilliSecs( time );
  pickUpController.SetCurrentTimeInMilliSecs( time );
  obstacleController.setCurrentTimeInMilliSecs( time );
  //searchController.SetCurrentTimeInMilliSecs( time);
  //random_dispersal_controller.setCurrentTime( time );
}

/*****
 * Return the Poisson cumulative probability at a given local resource density and lambda.
 *****/
/*double LogicController::PoissonCDF(double lambda)
{
  double sumAccumulator       = 1.0;
  double factorialAccumulator = 1.0;
  
  for (size_t i = 1; i <= local_resource_density; i++) {
    factorialAccumulator *= i;
    sumAccumulator += pow(lambda, i) / factorialAccumulator;
  }
  
  return (exp(-lambda) * sumAccumulator);
}
*/

/*void LogicController::setTargetHeld() 
{
  target_held = true;
}*/
void LogicController::senseLocalResourceDensity(int num_tags) {
	local_resource_density = num_tags;
  if(cpfa_state == sense_local_resource_density) {
    searchController.senseLocalResourceDensity(num_tags); //should be removed? qilu 11/2017
     dropOffController.senseLocalResourceDensity(num_tags);
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


void LogicController::SetModeAuto() {
  if(processState == PROCCESS_STATE_MANUAL) {
    // only do something if we are in manual mode
    cout<<"do something if we are in manual mode..."<<endl;
    this->Reset();
    manualWaypointController.Reset();
  }
}
void LogicController::SetModeManual()
{
  if(processState != PROCCESS_STATE_MANUAL) {
    logicState = LOGIC_STATE_INTERRUPT;
    processState = PROCCESS_STATE_MANUAL;
    ProcessData();
    control_queue = priority_queue<PrioritizedController>();
    driveController.Reset();
  }
}

