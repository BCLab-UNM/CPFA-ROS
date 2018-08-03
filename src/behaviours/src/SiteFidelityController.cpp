#include "SiteFidelityController.h"

SiteFidelityController::SiteFidelityController () 
{
  site_fidelity_location.x = 0;
  site_fidelity_location.y = 0;
  site_fidelity_location.theta = 0;

  current_location.x = 0;
  current_location.y = 0;
  current_location.theta = 0;
}

SiteFidelityController::~SiteFidelityController () {}

void SiteFidelityController::Reset()
{
}


void SiteFidelityController::SiteFidelityReset()
{
	site_fidelity_location.x = 0;
    site_fidelity_location.y = 0;
    site_fidelity_location.theta = 0;
}
	  
	  
Result SiteFidelityController::DoWork() 
{
	//cout<<"CPFAStatus: SiteFidelityController::DoWork()"<<endl;
  Result result;
  cout <<"wpTest: site_fidelity_location="<<site_fidelity_location.x<<", "<<site_fidelity_location.y<<endl;
  //cout <<"SF: current_location="<<current_location.x<<", "<<current_location.y<<endl;
          
  if (hypot(site_fidelity_location.x - current_location.x, site_fidelity_location.y - current_location.y) < 0.15 || attemptCount>=ATTEMPT_MAX) 
  {
	  cout <<"wpTest: SF: Reached site fidelity"<<endl;
	  attemptCount=0;
	SiteFidelityReset();  
    result.type = behavior;
    result.b = COMPLETED;
    cout <<"result.wpts.waypoints size="<<result.wpts.waypoints.size()<<endl;
    if (attemptCount >=ATTEMPT_MAX)
    {
		cout <<"wpTest: give up to go to site fidelity and start to search..."<<endl;
		}
  } 

  else if(attemptCount<ATTEMPT_MAX)
  {
	  attemptCount++;
	  cout<<"wpTest: SF: travel to site fidelity, attemptCount="<<attemptCount<<endl; 
    result.type = waypoint;
    result.PIDMode = FAST_PID;
    result.wpts.waypoints.insert(result.wpts.waypoints.begin(), site_fidelity_location);
    /*cout <<"TestStatus: result.wpts.waypoints size="<<result.wpts.waypoints.size()<<endl;
    for(int i=0; i<result.wpts.waypoints.size(); i++){
		cout<<"TestStatus: result wpts wp["<<i<<"]="<<result.wpts.waypoints[i].x<<", "<<result.wpts.waypoints[i].y<<"]"<<endl;
		}*/
  }

  return result;
}

CPFAState SiteFidelityController::GetCPFAState() 
{
  return cpfa_state;
}

void SiteFidelityController::SetCPFAState(CPFAState state) {
  cpfa_state = state;
  result.cpfa_state = state;
}

bool SiteFidelityController::ShouldInterrupt()
{
 //cout<<"site fidelity controller should interrupt..."<<endl;	
  ProcessData();

  return false;
}

bool SiteFidelityController::HasWork() 
{
  return true;
}

void SiteFidelityController::SetTargetPickedUp()
{
  target_picked_up = true;
  ProcessData();//commented in previous version
}

void SiteFidelityController::SetCurrentLocation(Point current_location)
{
  this->current_location = current_location;
}

bool SiteFidelityController::SiteFidelityInvalid()
{
	if(site_fidelity_location.x == 0 && site_fidelity_location.y == 0)
	{
		return true;
		
	}
		
	return false; 
}

void SiteFidelityController::ProcessData() 
{
  if (target_picked_up)
  {
	  cout<<"TestStatus: Create site fidelity...["<<current_location.x<<", "<<current_location.y<<"]"<<endl;
    site_fidelity_location = current_location;
    
    target_picked_up = false;
  }
}

