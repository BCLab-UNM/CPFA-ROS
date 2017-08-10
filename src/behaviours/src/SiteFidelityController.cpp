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

Result SiteFidelityController::DoWork() 
{
  
  Result result;

  if (hypot(site_fidelity_location.x - current_location.x, site_fidelity_location.y - current_location.y) < 0.15) 
  {
    result.type = behavior;
    result.b = COMPLETED;
  } 

  else 
  {
    result.type = waypoint;
    result.PIDMode = FAST_PID;
    result.wpts.waypoints.insert(result.wpts.waypoints.begin(), site_fidelity_location);
  }

  return result;
}

bool SiteFidelityController::ShouldInterrupt()
{
  ProcessData();

  return false;
}

bool SiteFidelityController::HasWork() 
{
  return true;
}

void SiteFidelityController::setTargetPickedUp()
{
  target_picked_up = true;
}

void SiteFidelityController::setCurrentLocation(Point current_location)
{
  this->current_location = current_location;
}

void SiteFidelityController::ProcessData() 
{
  if (target_picked_up)
  {
    site_fidelity_location = current_location;
    target_picked_up = false;
  }
}
