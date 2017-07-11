#include <ros/ros.h>
#include "CPFASearchController.h"


// Public
CPFASearchController::CPFASearchController(){}

CPFASearchController::CPFASearchController(std::string published_name) 
{
  arena_size = 15;
  travel_step_size = 0.5;
  search_step_size = 0.5;
  min_distance_to_target = 1;

  search_state = start_state;
  search_location_type = random_search;
  rng = new random_numbers::RandomNumberGenerator();
  rover_name = published_name;
}

/**
 * Continues search pattern after interruption. For example, avoiding the
 * center or collisions.
 */
geometry_msgs::Pose2D CPFASearchController::continueInterruptedSearch(const geometry_msgs::Pose2D& current_location,
                                                                      const geometry_msgs::Pose2D& old_goal_location,
                                                                      const geometry_msgs::Pose2D& center_location)
{

  // If the rover is near the edge of the arena, start searching
  if(search_state == travel_to_search_site && distanceToLocation(current_location, center_location) > (arena_size/2 - travel_step_size - 0.5)){
    ROS_INFO_STREAM(rover_name << "CPFA: continueInterruptedSearch while traveling to search site");

    if(search_location_type == site_fidelity || search_location_type == pheromone) {

      search_state = search_with_informed_walk;
      local_resource_density = 0;
      informed_search_start_time = ros::Time::now();
      ROS_INFO_STREAM(rover_name << "CPFA: search_with_informed_walk");
    } else {

      search_state = search_with_uninformed_walk;
      ROS_INFO_STREAM(rover_name << "CPFA: search_with_uninformed_walk");
    }

    return CPFAStateMachine(current_location, center_location);

  }else{
    geometry_msgs::Pose2D new_goal_location;

    //remainingGoalDist avoids magic numbers by calculating the dist
    double remainingGoalDist = hypot(old_goal_location.x - current_location.x, old_goal_location.y - current_location.y);

    //this of course assumes random walk continuation. Change for diffrent search methods.
    new_goal_location.theta = old_goal_location.theta;
    new_goal_location.x = current_location.x + (0.50 * cos(old_goal_location.theta)); //(remainingGoalDist * cos(old_goal_location.theta));
    new_goal_location.y = current_location.y + (0.50 * sin(old_goal_location.theta)); //(remainingGoalDist * sin(old_goal_location.theta));

    return new_goal_location;
  }
}

/**
 * This function decides which state function to call based on the value
 * of the search_state variable.
 */
geometry_msgs::Pose2D CPFASearchController::CPFAStateMachine(const geometry_msgs::Pose2D& current_location,
                                                             const geometry_msgs::Pose2D& center_location)
{
  updatePheromoneList();

  switch(search_state) {
    // Initializes CPFA parameters
    case start_state:
      start();

      // Sets target location to a site fidelity location, pheromone, or random location.
      // Preset when rover has picked up a resource and is returning to collection zone
    case set_search_location:
      setSearchLocation(current_location, center_location);
      break;

      // Travels to target location while ignoring resources
    case travel_to_search_site:
      travelToSearchSite(current_location);
      break;

      // Searches at a random target location
    case search_with_uninformed_walk:
      searchWithUninformedWalk(current_location, center_location);
      break;

      // Searches at a site fidelity location or pheromone location
    case search_with_informed_walk:
      searchWithInformedWalk(current_location, center_location);
      break;

      // Counts local resource density while picking up a resource
    case sense_local_resource_density:
      break;

      // Only set when Rover has given up search
    case return_to_nest:
      returnToNest(current_location);
  }

  return target_location; // will place into Result struct later
}

void CPFASearchController::senseLocalResourceDensity(const int& num_trags)
{
  if(num_trags > local_resource_density){
    local_resource_density = num_trags;
    ROS_INFO_STREAM(rover_name << "CPFA: local resource density: " << local_resource_density);
  }
}

bool CPFASearchController::layPheromone() 
{
  double poisson = getPoissonCDF(rate_of_laying_pheromone);
  double random_num = rng->uniformReal(0, 1);

  ROS_INFO_STREAM(rover_name << "CPFA: layPheromone, poisson: " << poisson << " > random number: " << random_num);
  if(poisson > random_num)
    return true;
  else
    return false;
}

void CPFASearchController::insertPheromone(const std::vector<geometry_msgs::Pose2D>& pheromone_trail)
{
  // At this point in time we are not making a pheromone trail
  // the first index of the trail is the same position as the pheromone location
  geometry_msgs::Pose2D newLocation = pheromone_trail[0];
  Pheromone pheromone(newLocation, pheromone_trail, ros::Time::now(), rate_of_pheromone_decay);

  pheromones.push_back(pheromone);
  ROS_INFO_STREAM(rover_name << "CPFA: inserting pheromone pheromones.size(): " << pheromones.size());
  ROS_INFO_STREAM(rover_name << "CPFA: pheromoneLocation x: " << pheromone_trail[0].x << " y: " << pheromone_trail[0].y);
}


CPFAState CPFASearchController::getState() 
{
  return search_state;
}

void CPFASearchController::setState(const CPFAState& state) {
  if(state == set_search_location)
    ROS_INFO_STREAM(rover_name << "CPFA: set_search_location");
  else if(state == travel_to_search_site)
    ROS_INFO_STREAM(rover_name << "CPFA: travel_to_search_site");
  else if(state == search_with_uninformed_walk)
    ROS_INFO_STREAM(rover_name << "CPFA: search_with_uninformed_walk");
  else if(state == search_with_informed_walk)
    ROS_INFO_STREAM(rover_name << "CPFA: search_with_informed_walk");
  else if(state == sense_local_resource_density)
    ROS_INFO_STREAM(rover_name << "CPFA: sense_local_resource_density");
  else
    ROS_INFO_STREAM(rover_name << "CPFA: return_to_nest");

  // If true, rover failed to pickup block and is restarting search
  if(state == search_with_informed_walk || state == search_with_uninformed_walk){
    local_resource_density = 0;
    informed_search_start_time = ros::Time::now();
  }

  search_state = state;
}

SearchLocationType CPFASearchController::getSearchLocationType() 
{
  return search_location_type;
}

void CPFASearchController::setSearchLocationType(const SearchLocationType& type)
{
  if(type == site_fidelity)
    ROS_INFO_STREAM(rover_name << "CPFA: search_location_type site_fidelity");
  else if(type == pheromone)
    ROS_INFO_STREAM(rover_name << "CPFA: search_location_type pheromone");
  else
    ROS_INFO_STREAM(rover_name << "CPFA: search_location_type random_search");

  search_location_type = type;
}

geometry_msgs::Pose2D CPFASearchController::getTargetLocation()
{
  return target_location;
}

void CPFASearchController::setTargetLocation(const geometry_msgs::Pose2D& site_location,
                                             const geometry_msgs::Pose2D& center_location)
{
  target_location.theta = atan2(target_location.y - center_location.y, target_location.x - center_location.x);
  target_location.x = site_location.x;
  target_location.y = site_location.y;

  ROS_INFO_STREAM(rover_name << "CPFA: setTargetLocation  target_location.x: " << target_location.x << " target_location.y: " << target_location.y);
}



// Private
void CPFASearchController::start() 
{
  ROS_INFO_STREAM(rover_name << "CPFA: start_state");

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

  search_state = set_search_location;
  ROS_INFO_STREAM(rover_name << "CPFA: set_search_location");
}

void CPFASearchController::setSearchLocation(const geometry_msgs::Pose2D& current_location,
                                             const geometry_msgs::Pose2D& center_location)
{
  search_state = travel_to_search_site;
  ROS_INFO_STREAM(rover_name << "CPFA: travel_to_search_site");

  local_resource_density = 0;

  if(search_location_type == site_fidelity){
    double poisson = getPoissonCDF(rate_of_site_fidelity);
    double random_num = rng->uniformReal(0, 1);

    ROS_INFO_STREAM(rover_name << "CPFA: siteFidelity, poisson: " << poisson << " > random number: " << random_num);
    if(poisson > random_num) {
      ROS_INFO_STREAM(rover_name << "CPFA: SearchLocationType site_fidelity");

      // Leave target_location to previous block location
      // Adjust heading so rover headers to target
      target_location.theta = atan2(target_location.y - current_location.y, target_location.x - current_location.x);
      return;
    } else {
      search_location_type = pheromone;
    }
  }

  if(search_location_type == pheromone && pheromones.size() > 0){
    ROS_INFO_STREAM(rover_name << "CPFA: SearchLocationType pheromone pheromones.size(): " << pheromones.size());
    setPheromone(center_location);

    // Adjust heading so rover headers to target
    target_location.theta = atan2(target_location.y - current_location.y, target_location.x - current_location.x);
    return;
  }

  search_location_type = random_search;
  ROS_INFO_STREAM(rover_name << "CPFA: SearchLocationType random_search");

  target_location.theta = rng->uniformReal(0, 2 * M_PI);
  travelToSearchSite(current_location);
}

void CPFASearchController::travelToSearchSite(const geometry_msgs::Pose2D& current_location)
{

  if(search_location_type == site_fidelity || search_location_type == pheromone) {
    // Reached site fidelity location and switch to informed search

    if(distanceToLocation(current_location, target_location) < min_distance_to_target) {
      informed_search_start_time = ros::Time::now();
      local_resource_density = 0;
      search_state = search_with_informed_walk;
      ROS_INFO_STREAM(rover_name << "CPFA: search_with_informed_walk");
    }


    // Heading to site fidelity, keep target location the same
    // Adjust heading so rover headers to target
    target_location.theta = atan2(target_location.y - current_location.y, target_location.x - current_location.x);
    return;
  }

  // search_location_type is random_search
  if(rng->uniformReal(0, 1) < probability_of_switching_to_searching){
    search_state = search_with_uninformed_walk;
    ROS_INFO_STREAM(rover_name << "CPFA: search_with_uninformed_walk");
  }

  target_location.x = current_location.x + travel_step_size*cos(target_location.theta);
  target_location.y = current_location.y + travel_step_size*sin(target_location.theta);
}

void CPFASearchController::searchWithUninformedWalk(const geometry_msgs::Pose2D& current_location,
                                                    const geometry_msgs::Pose2D& center_location)
{
  if(giveUpSearching(current_location, center_location))
    return;

  target_location.theta = rng->gaussian(current_location.theta, uninformed_search_variation);
  target_location.x = current_location.x + search_step_size*cos(target_location.theta);
  target_location.y = current_location.y + search_step_size*sin(target_location.theta);
}

void CPFASearchController::searchWithInformedWalk(const geometry_msgs::Pose2D& current_location, 
                                                  const geometry_msgs::Pose2D& center_location) 
{
  if(giveUpSearching(current_location, center_location))
    return;

  double correlation = calculateInformedWalkCorrelation();

  target_location.theta = rng->gaussian(current_location.theta, correlation); 
  target_location.x = current_location.x + search_step_size*cos(target_location.theta);
  target_location.y = current_location.y + search_step_size*sin(target_location.theta);
}

void CPFASearchController::returnToNest(const geometry_msgs::Pose2D& current_location)
{
  if(distanceToLocation(current_location, target_location) < min_distance_to_target) {
    search_state = set_search_location;
    ROS_INFO_STREAM(rover_name << "CPFA: set_search_location");
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

double CPFASearchController::distanceToLocation(const geometry_msgs::Pose2D& L1, const geometry_msgs::Pose2D& L2)
{
  return hypot(L1.x - L2.x, L1.y - L2.y);
}

double CPFASearchController::calculateInformedWalkCorrelation()
{
  double search_time = (ros::Time::now() - informed_search_start_time).toSec();
  ROS_INFO_STREAM(rover_name << "CPFA: informedSearchTime: " << search_time);

  return uninformed_search_variation + (4 * M_PI - uninformed_search_variation) * exp(-rate_of_informed_search_decay * search_time);
}

bool CPFASearchController::giveUpSearching(const geometry_msgs::Pose2D& current_location,
                                           const geometry_msgs::Pose2D& center_location) 
{
  double random_num = rng->uniformReal(0, 1);

  if(probability_of_returning_to_nest > random_num)  {
    search_location_type = pheromone;
    search_state = return_to_nest;
    ROS_INFO_STREAM(rover_name << "CPFA: return_to_nest");

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
    geometry_msgs::Pose2D pLoc = pheromones[i].getLocation();
    pheromones[i].update(time);

    if(pheromones[i].isActive()) {
      newPheromoneList.push_back(pheromones[i]);
    }
  }


  pheromones = newPheromoneList;
}

void CPFASearchController::setPheromone(const geometry_msgs::Pose2D& center_location)
{
  ROS_INFO_STREAM(rover_name << "CPFA: Setting pheromone location...");;
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
      ROS_INFO_STREAM(rover_name << "CPFA: pheromoneLocation x: " << target_location.x << " y: " << target_location.y);

      // TrailToFollow = pheromones[i].GetTrail();

      //If we pick a pheromone, break out of this loop. 
      break;
    }

    //We didn't pick a pheromone! Remove its weight from randomWeight. 
    randomWeight -= pheromones[i].getWeight();
  }
}

void CPFASearchController::setArenaSize(const int& numRovers) 
{
  // Num rovers is every other rover, hence >= 3
  ROS_INFO_STREAM(rover_name << "CPFA: numRovers: " << numRovers);
  if(numRovers > 3) {
    arena_size = 22;
  }
}
