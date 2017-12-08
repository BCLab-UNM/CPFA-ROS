#include "PheromoneController.h"

using namespace std;

PheromoneController::PheromoneController()
{
}

PheromoneController::~PheromoneController()
{
}

void PheromoneController::Reset()
{
  targetHeld = true;//qilu 12/2017
  sense_local_density_completed = false;//qilu 12/2017
  drive_to_pheromone =false; //qilu 12/2017
}

void PheromoneController::ProcessData() 
{
}


CPFAState PheromoneController::GetCPFAState() 
{
  return cpfa_state;
}

void PheromoneController::SetCPFAState(CPFAState state) 
{
  cpfa_state = state;
  result.cpfa_state = state;
}

bool PheromoneController::Sensing()
{
	return sense_local_density_completed;
	
	}
	
void PheromoneController::DriveToPheromoneTrail()
{
	drive_to_pheromone= true;
	}

Result PheromoneController::DoWork()
{
	cout<<"CPFAStatus: PheromoneController::DoWork()"<<endl;
  cout<<"CPFAStatus: targetHeld="<<targetHeld<<endl;
  if (targetHeld)
  {
    //sense_local_density_completed = false;
    targetHeld = false;
    time_searching = current_time/1e3;//no reference. should be removed. qilu 12/2017
    cout<<"time_searching="<<time_searching<<endl;
  }
  cout<<"current_time="<<current_time/1e3<<endl;
  int time_elapsed = current_time/1e3 - time_searching;
  cout <<"CPFAStatus: time_elapsed="<<time_elapsed<<endl;
  if (!sense_local_density_completed && !drive_to_pheromone)
  {
    if (time_elapsed > target_search_time)
    {
		cout<<"CPFAStatus: Pheromone: sense completed..."<<endl;
      result.type = behavior;
      //result.b = COMPLETED;
      result.b = nextProcess;
    result.reset = true;
    sense_local_density_completed = true;
    }
    else
    {
		cout<<"CPFAStatus: Sense local density..."<<endl;
      result.type = precisionDriving;
      result.PIDMode = CONST_PID;
      
      result.pd.cmdVel = 0.0;
      result.pd.cmdAngular = 2*M_PI/target_search_time; //2 PI radians divided by seconds to choose how long it takes to spin 360 degrees
    }
  }
  
  if(drive_to_pheromone)
  {
	  cout <<"CPFAStatus: dirve to pheromone..."<<endl;
	  selected_pheromone.x = 2.0;
	  selected_pheromone.y = 2.0;
	  cout<<"selected_pheromone.x="<<selected_pheromone.x<<endl;
	  cout<<"current_location.x="<<current_location.x<<endl;
    if (hypot(selected_pheromone.x - current_location.x, selected_pheromone.y - current_location.y) < 0.15) 
    {
	  drive_to_pheromone= false;
	  cout <<"CPFAStatus: Reached pheromone waypoint..."<<endl;
      result.type = behavior;
      result.b = COMPLETED;
      cout <<"result.wpts.waypoints size="<<result.wpts.waypoints.size()<<endl;
      for(int i=0; i<result.wpts.waypoints.size(); i++)
      {
		cout<<i<<".x= "<<result.wpts.waypoints[i].x<<endl;
		}
    
    } 
    else 
    {
	  cout <<"CPFAStatus: SF: travel to pheromone_waypoint, "<<selected_pheromone.x<<endl;
      result.type = waypoint;
      result.PIDMode = FAST_PID;
      result.wpts.waypoints.insert(result.wpts.waypoints.begin(), selected_pheromone);
      cout <<"result.wpts.waypoints size="<<result.wpts.waypoints.size()<<endl;
      for(int i=0; i<result.wpts.waypoints.size(); i++)
      {
		cout<<i<<".x= "<<result.wpts.waypoints[i].x<<endl;
		}
    }
  }
  
  return result;
}

bool PheromoneController::ShouldInterrupt()
{
  cout<<"pheromone controller should interrupt..."<<endl;
  //qilu 12/2017
  //ProcessData(); 
  if(sense_local_density_completed)
  {
    return true;
  }
  else
  {
    return false;
  }
  
}

bool PheromoneController::HasWork()
{
	cout<<"Pheromone has work...True"<<endl;
  bool has_work = true;
  
  return has_work;
  
}


void PheromoneController::setTargetDate(std::vector<Tag> tags)
{
  
  int target_count = 0;
  
  if (!sense_local_density_completed)
  {
    if (tags.size() > 0)
    {
      for (int i = 0; i < tags.size(); i++) 
      {
        if (tags[i].getID() == 0) 
        {
          target_count++;
        }
      }
    }
    
    if (resource_density < target_count) { resource_density = target_count; }
    
    
  }
}



