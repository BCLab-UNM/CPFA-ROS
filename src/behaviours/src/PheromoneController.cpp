#include "PheromoneController.h"

using namespace std;

PheromoneController::PheromoneController()
{
}

void PheromoneController::Reset()
{
  
}

Result PheromoneController::DoWork()
{
  
  if (targetHeld)
  {
    sense_local_density = true;
    targetHeld = false;
    time_searching = current_time;
  }
  
  int time_elapsed = 0;
  
  if (sense_local_density)
  {
    if (time_elapsed > target_search_time)
    {
      result.type = behavior;
      result.b = COMPLETED;
    }
    else
    {
      result.type = precisionDriving;
      result.PIDMode = CONST_PID;
      
      result.pd.cmdVel = 0.0;
      result.pd.cmdAngular = 2*M_PI/target_search_time; //2 PI radians divided by seconds to choose how long it takes to spin 360 degrees
    }
  }
  else //dirve to pheromone
  {
    
  }
  
  return result;
}

bool PheromoneController::ShouldInterrupt()
{
  
}

bool PheromoneController::HasWork()
{
  bool has_work = true;
  
  return has_work;
  
}


void PheromoneController::setTargetDate(std::vector<Tag> tags)
{
  
  int target_count = 0;
  
  if (sense_local_density)
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



