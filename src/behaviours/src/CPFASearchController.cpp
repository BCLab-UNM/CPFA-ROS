#include "CPFASearchController.h"

// PUBLIC Functions

CPFASearchController::CPFASearchController(string name) {
  this->name = name;

  arena_size = 15;
  travel_step_size = 0.5;
  search_step_size = 0.5;
  min_distance_to_target = 1;

  SetCPFAState(start_state);
  SetCPFASearchType(random_search);

  rng = new random_numbers::RandomNumberGenerator();
  result.PIDMode = FAST_PID;
}

void CPFASearchController::Reset() {
  result.reset = false;
}

/**
 * This code implements a basic random walk search.
 */
Result CPFASearchController::DoWork() {
  if(cpfa_state == set_target_location) {
    // Reset attempt count after returning to the nest
    attempt_count = 0;
  }

  if (!result.wpts.waypoints.empty()) {
    if (hypot(result.wpts.waypoints[0].x-current_location.x, result.wpts.waypoints[0].y-current_location.y) < 0.10) {
      attempt_count = 0;
    }
  }

  cout << "Attempt Count: " << attempt_count << endl;
  if (attempt_count > 0 && attempt_count < attempt_count_threshold) {
    attempt_count++;
    if (succesfull_pickup) {
      succesfull_pickup = false;
      attempt_count = 1;
    }

    if(avoided_obstacle) 
    {
      Point dodge_point;
      if(turn_direction)
      {
        // Just turned counterlockwise, set point to the left of the robot
        dodge_point.x = current_location.x + 0.5 * cos(current_location.theta + M_PI/4);
        dodge_point.y = current_location.y + 0.5 * sin(current_location.theta + M_PI/4);
      } else
      {
        // Just turned clockwise, set point to the right of the robot
        dodge_point.x = current_location.x + 0.5 * cos(current_location.theta - M_PI/4);
        dodge_point.y = current_location.y + 0.5 * sin(current_location.theta - M_PI/4);
      }

      cout << "current_location x: " << current_location.x << " y: " << current_location.y << " theta: " << current_location.theta << endl;
      cout << "dodge_point x: " << dodge_point.x << " y: " << dodge_point.y << endl;
      cout << "target_Location x: " << target_location.x << " y: " << target_location.y << endl;
      cout << "turn_direction: " << turn_direction << endl;
      result.wpts.waypoints.clear();
      if(cpfa_search_type != random_search) 
      {
        result.wpts.waypoints.push_back(target_location);
      }
      result.wpts.waypoints.push_back(dodge_point);
    }
    return result;
  }
  else if (attempt_count >= attempt_count_threshold || attempt_count == 0) {

    //Rover is traveling and is blocked by obstacle
    if(attempt_count >= attempt_count_threshold && cpfa_state == travel_to_search_site) {
      cout << "Interrupted travel" << endl;

      if(cpfa_search_type == site_fidelity || cpfa_search_type == pheromone) {

        SetCPFAState(search_with_informed_walk);
        local_resource_density = 0;
        informed_search_start_time = ros::Time::now();
        cout << "search_with_informed_walk" << endl;
      } else {

        SetCPFAState(search_with_uninformed_walk);
        cout << "search_with_uninformed_walk" << endl;
      }

    }

    attempt_count = 1;
    result.type = waypoint;

    // CPFA Logic
    updatePheromoneList();

    switch(cpfa_state) {
      // Initializes CPFA parameters
      case start_state:
        start();

        // Sets target location to a site fidelity location, pheromone, or random location.
        // Preset when rover has picked up a resource and is returning to collection zone
      case set_target_location:
        setSearchLocation();
        break;

        // Travels to target location while ignoring resources
      case travel_to_search_site:
        travelToSearchSite();
        break;

        // Searches at a random target location
      case search_with_uninformed_walk:
        searchWithUninformedWalk();
        break;

        // Searches at a site fidelity location or pheromone location
      case search_with_informed_walk:
        searchWithInformedWalk();
        break;

        // Counts local resource density while picking up a resource
      case sense_local_resource_density:
        break;

        // Only set when Rover has given up search
      case return_to_nest:
        returnToNest();
    }

    //select new position 50 cm from current location
    result.wpts.waypoints.clear();
    result.wpts.waypoints.insert(result.wpts.waypoints.begin(), target_location);
    cout << "target_location x: " << target_location.x << " y: " << target_location.y << " theta: " << target_location.theta << endl;

    return result;
  }

}

bool CPFASearchController::ShouldInterrupt(){
  ProcessData();

  return false;
}

bool CPFASearchController::HasWork() {
  return true;
}

void CPFASearchController::senseLocalResourceDensity(int num_tags)
{
  if(num_tags > local_resource_density){
    local_resource_density = num_tags;
  }

  cout << "ROSAdapter: targetHandler" << endl;
  cout << "local_resource_density: " << local_resource_density << endl;
  cout << endl;
}

bool CPFASearchController::layPheromone() 
{
  double poisson = getPoissonCDF(rate_of_laying_pheromone);
  double random_num = rng->uniformReal(0, 1);

  if(poisson > random_num) {
    cout << "Laying a pheromone..." << endl;
    cout << endl;
    return true;
  } else {
    return false;
  }

}

void CPFASearchController::insertPheromone(const vector<Point> &pheromone_trail)
{
  // At this point in time we are not making a pheromone trail
  // the first index of the trail is the same position as the pheromone location
  Point new_location = pheromone_trail[0];
  Pheromone pheromone(new_location, pheromone_trail, ros::Time::now(), rate_of_pheromone_decay);

  pheromones.push_back(pheromone);
  cout << "inserting pheromone pheromones.size(): " << pheromones.size() << endl;
  cout << "pheromoneLocation x: " << pheromone_trail[0].x << " y: " << pheromone_trail[0].y << endl;
  cout << "================================================================" << endl;
}

CPFAState CPFASearchController::GetCPFAState() 
{
  return cpfa_state;
}

void CPFASearchController::SetCPFAState(CPFAState state) {
  // If true, rover failed to pickup block and is restarting search
  if(state == search_with_informed_walk || state == search_with_uninformed_walk){
    local_resource_density = 0;
    informed_search_start_time = ros::Time::now();
  }

  cpfa_state = state;
  result.cpfa_state = state;
}


CPFASearchType CPFASearchController::GetCPFASearchType() 
{
  return cpfa_search_type;
}

void CPFASearchController::SetCPFASearchType(CPFASearchType type)
{
  cpfa_search_type = type;
  result.cpfa_search_type = type;
}

Point CPFASearchController::getTargetLocation()
{
  return target_location;
}

void CPFASearchController::setTargetLocation(const Point& site_location)
{
  target_location.theta = atan2(target_location.y - current_location.y, target_location.x - current_location.x);
  target_location.x = site_location.x;
  target_location.y = site_location.y;

  cout << "setTargetLocation  target_location.x: " << target_location.x << " target_location.y: " << target_location.y << endl;
}


void CPFASearchController::SetSuccesfullPickup() {
  succesfull_pickup = true;
}

void CPFASearchController::setArenaSize(int num_rovers) 
{
  cout << "num_rovers: " << num_rovers << endl;
  if(num_rovers > 3) {
    arena_size = 22;
  }
}

void CPFASearchController::SetCurrentLocation(Point current_location) {
  this->current_location = current_location;
}

void CPFASearchController::SetCenterLocation(Point center_location) {
  this->center_location = center_location;
}

// PROTECTED FUNCTIONS

void CPFASearchController::ProcessData() {
}

// PRIVATE FUNCTIONS

void CPFASearchController::start() 
{
  cout << "start_state" << endl;

  // Default CPFA parameters
  probability_of_switching_to_searching = 0.15; // Increasing grows the probability
  probability_of_returning_to_nest = 0.01; // Increasing grows the probability
  uninformed_search_variation = 0.4; // The change in heading using uninformed search
  rate_of_informed_search_decay = 1.0/6.0; // Inverse of the expected time to find a resource
  rate_of_site_fidelity = 1; // Lower grows the probability
  rate_of_laying_pheromone = 5; // Lower grows the probability
  rate_of_pheromone_decay = 1.0/40.0; // Inverse of expected pheromone time

  // Parameters set from the CPFA parameters yaml file
  //ros::param::get("/" + rover_name + "/CPFA/probability_of_switching_to_searching", probability_of_switching_to_searching);
  //ros::param::get("/" + rover_name + "/CPFA/probability_of_returning_to_nest", probability_of_returning_to_nest);
  //ros::param::get("/" + rover_name + "/CPFA/uninformed_search_variation", uninformed_search_variation);
  //ros::param::get("/" + rover_name + "/CPFA/rate_of_informed_search_decay", rate_of_informed_search_decay);
  //ros::param::get("/" + rover_name + "/CPFA/rate_of_site_fidelity", rate_of_site_fidelity);
  //ros::param::get("/" + rover_name + "/CPFA/rate_of_laying_pheromone", rate_of_laying_pheromone);
  //ros::param::get("/" + rover_name + "/CPFA/rate_of_pheromone_decay", rate_of_pheromone_decay);

  SetCPFAState(set_target_location);
}

void CPFASearchController::setSearchLocation()
{
  SetCPFAState(travel_to_search_site);

  local_resource_density = 0;

  if(cpfa_search_type == site_fidelity){
    double poisson = getPoissonCDF(rate_of_site_fidelity);
    double random_num = rng->uniformReal(0, 1);

    cout << "siteFidelity, poisson: " << poisson << " > random number: " << random_num << endl;

    if(poisson > random_num) {
      // Leave target_location to previous block location
      // Adjust heading so rover headers to target
      target_location.theta = atan2(target_location.y - current_location.y, target_location.x - current_location.x);
      return;
    } else {
      SetCPFASearchType(pheromone);
    }

  }

  if(cpfa_search_type == pheromone && pheromones.size() > 0){
    cout << "CPFASearchType pheromone pheromones.size(): " << pheromones.size() << endl;
    setPheromone(center_location);

    // Adjust heading so rover headers to target
    target_location.theta = atan2(target_location.y - current_location.y, target_location.x - current_location.x);
    return;
  } else {
    cout << endl;
    cout << "Failed to use a pheromone!" << endl;
    cout << "CPFASearchType: " << cpfa_search_type << endl;
    cout << "Pheromones size: " << pheromones.size() << endl;
    cout << endl;
  }

  SetCPFASearchType(random_search);

  target_location.theta = rng->uniformReal(0, 2 * M_PI);
  travelToSearchSite();
}

void CPFASearchController::travelToSearchSite()
{

  if(cpfa_search_type == site_fidelity || cpfa_search_type == pheromone) {
    // Reached site fidelity location and switch to informed search

    if(distanceToLocation(current_location, target_location) < min_distance_to_target) {
      informed_search_start_time = ros::Time::now();
      local_resource_density = 0;
      SetCPFAState(search_with_informed_walk);
    }


    // Heading to site fidelity, keep target location the same
    // Adjust heading so rover headers to target
    target_location.theta = atan2(target_location.y - current_location.y, target_location.x - current_location.x);
    return;
  }

  // cpfa_search_type is random_search
  if(rng->uniformReal(0, 1) < probability_of_switching_to_searching){
    SetCPFAState(search_with_uninformed_walk);
  }

  target_location.x = current_location.x + travel_step_size*cos(target_location.theta);
  target_location.y = current_location.y + travel_step_size*sin(target_location.theta);
}

void CPFASearchController::searchWithUninformedWalk()
{
  if(giveUpSearching(current_location, center_location))
    return;

  target_location.theta = rng->gaussian(current_location.theta, uninformed_search_variation);
  target_location.x = current_location.x + search_step_size*cos(target_location.theta);
  target_location.y = current_location.y + search_step_size*sin(target_location.theta);

}

void CPFASearchController::searchWithInformedWalk()
{
  if(giveUpSearching(current_location, center_location))
    return;

  double correlation = calculateInformedWalkCorrelation();

  target_location.theta = rng->gaussian(current_location.theta, correlation); 
  target_location.x = current_location.x + search_step_size*cos(target_location.theta);
  target_location.y = current_location.y + search_step_size*sin(target_location.theta);
}

void CPFASearchController::returnToNest()
{
  if(distanceToLocation(current_location, target_location) < min_distance_to_target) {
    SetCPFAState(set_target_location);
  }
}

double CPFASearchController::getPoissonCDF(const double lambda)
{
  double sumAccumulator       = 1.0;
  double factorialAccumulator = 1.0;

  for (size_t i = 1; i <= local_resource_density; i++) {
    factorialAccumulator *= i;
    sumAccumulator += pow(lambda, i) / factorialAccumulator;
  }

  return (exp(-lambda) * sumAccumulator);
}

double CPFASearchController::distanceToLocation(const Point& L1, const Point& L2)
{
  return hypot(L1.x - L2.x, L1.y - L2.y);
}

double CPFASearchController::calculateInformedWalkCorrelation()
{
  double search_time = (ros::Time::now() - informed_search_start_time).toSec();
  cout << "informedSearchTime: " << search_time << endl;

  return uninformed_search_variation + (4 * M_PI - uninformed_search_variation) * exp(-rate_of_informed_search_decay * search_time);
}

bool CPFASearchController::giveUpSearching(const Point& current_location,
                                           const Point& center_location) 
{
  double random_num = rng->uniformReal(0, 1);

  if(probability_of_returning_to_nest > random_num)  {
    SetCPFAState(return_to_nest);
    SetCPFASearchType(pheromone);

    // Sets the target location 1 meter radially out directly from the center of the nest
    target_location.theta = atan2(center_location.y - current_location.y, center_location.x - current_location.x);
    target_location.x = center_location.x;
    target_location.y = center_location.y;

    return true;
  }  else {
    return false;
  }
}

void CPFASearchController::updatePheromoneList()
{
  ros::Time time = ros::Time::now();
  vector<Pheromone> newPheromoneList;

  for(int i = 0; i < pheromones.size(); i++) {
    Point pLoc = pheromones[i].getLocation();
    pheromones[i].update(time);

    if(pheromones[i].isActive()) {
      newPheromoneList.push_back(pheromones[i]);
    }
  }


  pheromones = newPheromoneList;
}

void CPFASearchController::setPheromone(const Point& center_location)
{
  cout << "Setting pheromone location..." << endl;
  double maxStrength = 0.0;
  double randomWeight = 0.0;

  //Calculate a maximum strength based on active pheromone weights.
  for(int i = 0; i < pheromones.size(); i++) {
    if(pheromones[i].isActive()) {
      maxStrength += pheromones[i].getWeight();
    }
  }
  // Calculate a random weight.
  randomWeight = rng->uniformReal(0.0, maxStrength);

  //Randomly select an active pheromone to follow. 
  for(int i = 0; i < pheromones.size(); i++) {

    if(randomWeight < pheromones[i].getWeight()) {
      //We've chosen a pheromone! 

      //  SetTarget(pheromones[i].GetLocation());
      target_location = pheromones[i].getLocation();
      cout << "pheromoneLocation x: " << target_location.x << " y: " << target_location.y << endl;

      // TrailToFollow = pheromones[i].GetTrail();

      //If we pick a pheromone, break out of this loop. 
      break;
    }

    //We didn't pick a pheromone! Remove its weight from randomWeight. 
    randomWeight -= pheromones[i].getWeight();
  }
}

void CPFASearchController::setObstacleAvoidance(bool turn_direction)
{
  avoided_obstacle = true;
  this->turn_direction = turn_direction;
}
