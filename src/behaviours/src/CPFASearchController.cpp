#include "CPFASearchController.h"

// PUBLIC Functions

CPFASearchController::CPFASearchController() {
  arena_size = 15;
  travel_step_size = 0.5;
  search_step_size = 0.5;
  min_distance_to_target = 1;

  cpfa_state = start_state;
  cpfa_search_type = random_search;
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

  if (!result.wpts.waypoints.empty()) {
    if (hypot(result.wpts.waypoints[0].x-current_location.x, result.wpts.waypoints[0].y-current_location.y) < 0.10) {
      attempt_count = 0;
    }
  }

  if (attempt_count > 0 && attempt_count < 5) {
    attempt_count++;
    if (succesfull_pickup) {
      succesfull_pickup = false;
      attempt_count = 1;
    }
    return result;
  }
  else if (attempt_count >= 5 || attempt_count == 0) {

    // Rover is traveling and is blocked by obstacle
    if(attempt_count >= 5 && cpfa_state == travel_to_search_site) {
      std::cout << "Interrupted travel" << std::endl;

      if(cpfa_search_type == site_fidelity || cpfa_search_type == pheromone) {

        cpfa_state = search_with_informed_walk;
        local_resource_density = 0;
        informed_search_start_time = ros::Time::now();
        std::cout << "search_with_informed_walk" << std::endl;
      } else {

        cpfa_state = search_with_uninformed_walk;
        std::cout << "search_with_uninformed_walk" << std::endl;
      }

    }

    attempt_count = 1;
    result.type = waypoint;

    // CPFA Logic
    updatePheromoneList();

    //switch(cpfa_state) {
      //// Initializes CPFA parameters
      //case start_state:
        //start();

        //// Sets target location to a site fidelity location, pheromone, or random location.
        //// Preset when rover has picked up a resource and is returning to collection zone
      //case set_target_location:
        //setSearchLocation();
        //break;

        //// Travels to target location while ignoring resources
      //case travel_to_search_site:
        //travelToSearchSite();
        //break;

        //// Searches at a random target location
      //case search_with_uninformed_walk:
        //searchWithUninformedWalk();
        //break;

        //// Searches at a site fidelity location or pheromone location
      //case search_with_informed_walk:
        //searchWithInformedWalk();
        //break;

        //// Counts local resource density while picking up a resource
      //case sense_local_resource_density:
        //break;

        //// Only set when Rover has given up search
      //case return_to_nest:
        //returnToNest();
    //}

    //select new position 50 cm from current location
    result.wpts.waypoints.clear();
    result.wpts.waypoints.insert(result.wpts.waypoints.begin(), target_location);

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
    std::cout << "local resource density: " << local_resource_density << std::endl;
  }
}

bool CPFASearchController::layPheromone() 
{
  double poisson = getPoissonCDF(rate_of_laying_pheromone);
  double random_num = rng->uniformReal(0, 1);

  std::cout << "layPheromone, poisson: " << poisson << " > random number: " << random_num << std::endl;
  if(poisson > random_num)
    return true;
  else
    return false;
}

void CPFASearchController::insertPheromone(const std::vector<Point> &pheromone_trail)
{
  // At this point in time we are not making a pheromone trail
  // the first index of the trail is the same position as the pheromone location
  Point new_location = pheromone_trail[0];
  Pheromone pheromone(new_location, pheromone_trail, ros::Time::now(), rate_of_pheromone_decay);

  pheromones.push_back(pheromone);
  std::cout << "inserting pheromone pheromones.size(): " << pheromones.size() << std::endl;
  std::cout << "pheromoneLocation x: " << pheromone_trail[0].x << " y: " << pheromone_trail[0].y << std::endl;
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
}


CPFASearchType CPFASearchController::GetCPFASearchType() 
{
  return cpfa_search_type;
}

void CPFASearchController::SetCPFASearchType(CPFASearchType type)
{
  cpfa_search_type = type;
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

  std::cout << "setTargetLocation  target_location.x: " << target_location.x << " target_location.y: " << target_location.y << std::endl;
}


void CPFASearchController::SetSuccesfullPickup() {
  succesfull_pickup = true;
}

void CPFASearchController::setArenaSize(int num_rovers) 
{
  std::cout << "num_rovers: " << num_rovers << std::endl;
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
  std::cout << "start_state" << std::endl;

  // Default CPFA parameters
  probability_of_switching_to_searching = 0.13; // Increasing grows the probability
  probability_of_returning_to_nest = 0.0; // Increasing grows the probability
  uninformed_search_variation = 0.4; // The change in heading using uninformed search
  rate_of_informed_search_decay = 1.0/6.0; // Inverse of the expected time to find a resource
  rate_of_site_fidelity = 0.3; // Lower grows the probability
  rate_of_laying_pheromone = 5; // Lower grows the probability
  rate_of_pheromone_decay = 1.0/40.0; // Inverse of expected pheromone time

  // Parameters set from the CPFA parameters yaml file
  ros::param::get("/" + rover_name + "/CPFA/probability_of_switching_to_searching", probability_of_switching_to_searching);
  ros::param::get("/" + rover_name + "/CPFA/probability_of_returning_to_nest", probability_of_returning_to_nest);
  ros::param::get("/" + rover_name + "/CPFA/uninformed_search_variation", uninformed_search_variation);
  ros::param::get("/" + rover_name + "/CPFA/rate_of_informed_search_decay", rate_of_informed_search_decay);
  ros::param::get("/" + rover_name + "/CPFA/rate_of_site_fidelity", rate_of_site_fidelity);
  ros::param::get("/" + rover_name + "/CPFA/rate_of_laying_pheromone", rate_of_laying_pheromone);
  ros::param::get("/" + rover_name + "/CPFA/rate_of_pheromone_decay", rate_of_pheromone_decay);

  cpfa_state = set_target_location;
  std::cout << "set_target_location" << std::endl;
}

void CPFASearchController::setSearchLocation()
{
  cpfa_state = travel_to_search_site;
  std::cout << "travel_to_search_site" << std::endl;

  local_resource_density = 0;

  if(cpfa_search_type == site_fidelity){
    double poisson = getPoissonCDF(rate_of_site_fidelity);
    double random_num = rng->uniformReal(0, 1);

    std::cout << "siteFidelity, poisson: " << poisson << " > random number: " << random_num << std::endl;
    if(poisson > random_num) {
      std::cout << "CPFASearchType site_fidelity" << std::endl;

      // Leave target_location to previous block location
      // Adjust heading so rover headers to target
      target_location.theta = atan2(target_location.y - current_location.y, target_location.x - current_location.x);
      return;
    } else {
      cpfa_search_type = pheromone;
    }
  }

  if(cpfa_search_type == pheromone && pheromones.size() > 0){
    std::cout << "CPFASearchType pheromone pheromones.size(): " << pheromones.size() << std::endl;
    setPheromone(center_location);

    // Adjust heading so rover headers to target
    target_location.theta = atan2(target_location.y - current_location.y, target_location.x - current_location.x);
    return;
  }

  cpfa_search_type = random_search;
  std::cout << "CPFASearchType random_search" << std::endl;

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
      cpfa_state = search_with_informed_walk;
      std::cout << "search_with_informed_walk" << std::endl;
    }


    // Heading to site fidelity, keep target location the same
    // Adjust heading so rover headers to target
    target_location.theta = atan2(target_location.y - current_location.y, target_location.x - current_location.x);
    return;
  }

  // cpfa_search_type is random_search
  if(rng->uniformReal(0, 1) < probability_of_switching_to_searching){
    cpfa_state = search_with_uninformed_walk;
    std::cout << "search_with_uninformed_walk" << std::endl;
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
    cpfa_state = set_target_location;
    std::cout << "set_target_location" << std::endl;
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
  std::cout << "informedSearchTime: " << search_time << std::endl;

  return uninformed_search_variation + (4 * M_PI - uninformed_search_variation) * exp(-rate_of_informed_search_decay * search_time);
}

bool CPFASearchController::giveUpSearching(const Point& current_location,
                                           const Point& center_location) 
{
  double random_num = rng->uniformReal(0, 1);

  if(probability_of_returning_to_nest > random_num)  {
    cpfa_search_type = pheromone;
    cpfa_state = return_to_nest;
    std::cout << "return_to_nest" << std::endl;

    // Sets the target location 1 meter radially out directly from the center of the nest
    target_location.theta = atan2(center_location.y - current_location.y, center_location.x - current_location.x);
    target_location.x = center_location.x - cos(target_location.theta);
    target_location.y = center_location.y - sin(target_location.theta);

    return true;
  }  else {
    return false;
  }
}

void CPFASearchController::updatePheromoneList()
{
  ros::Time time = ros::Time::now();
  std::vector<Pheromone> newPheromoneList;

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
  std::cout << "Setting pheromone location..." << std::endl;
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
      std::cout << "pheromoneLocation x: " << target_location.x << " y: " << target_location.y << std::endl;

      // TrailToFollow = pheromones[i].GetTrail();

      //If we pick a pheromone, break out of this loop. 
      break;
    }

    //We didn't pick a pheromone! Remove its weight from randomWeight. 
    randomWeight -= pheromones[i].getWeight();
  }
}

