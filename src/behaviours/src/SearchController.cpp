#include "SearchController.h"
#include <angles/angles.h>

using namespace std;

SearchController::SearchController() {
  //pheromones.clear(); //qilu 08/2017
  rng = new random_numbers::RandomNumberGenerator();
  currentLocation.x = 0;
  currentLocation.y = 0;
  currentLocation.theta = 0;

  centerLocation.x = 0;
  centerLocation.y = 0;
  centerLocation.theta = 0;
  result.PIDMode = FAST_PID;

  result.fingerAngle = M_PI/2;
  result.wristAngle = M_PI/4;
}

void SearchController::Reset() {
	cout<<"SearchController::Reset()"<<endl;
  result.reset = false;
  attemptCount = 0;
  result.wpts.waypoints.clear();
}
	
	
/**
 * This code implements a basic random walk search.
 */
Result SearchController::DoWork() {
  cout<<"SF: CPFAStatus: SearchController::DoWork(), size="<<result.wpts.waypoints.size()<<endl;
  //cout<<"search 1. result.wpts.waypoints size ="<<result.wpts.waypoints.size()<<endl;
  if (!result.wpts.waypoints.empty()) {
	  cout<<"CPFAStatus: result.wpts.waypoints="<<result.wpts.waypoints[0].x<<"  "<<result.wpts.waypoints.size()<<endl;
	  cout <<"waypoint is not empty..."<<endl;
    if (hypot(result.wpts.waypoints[0].x-currentLocation.x, result.wpts.waypoints[0].y-currentLocation.y) < 0.15) {
      cout<<"CPFAStatus: reach waypoint..."<<endl;
      result.wpts.waypoints.clear();
      attemptCount = 0;
    }
  }
  cout <<"CPFAStatus: attemptCount="<<attemptCount<<endl;
  if (attemptCount > 0 && attemptCount < 5) {
    attemptCount++;
    if (succesfullPickup) {
      succesfullPickup = false;
      attemptCount = 1;
    }
    return result;
  }
  else if (attemptCount >= 5 || attemptCount == 0) {
    attemptCount = 1;


    result.type = waypoint;
    Point  searchLocation;

    
    float correlation = uninformed_search_variation;
      
    if (informed_search)
    {
      cout << "CPFAStatus: Informed Search" << endl;
      float exponential = exp(-rate_of_informed_search_decay * informed_search_time);
      exponential *= (4 * M_PI - uninformed_search_variation);
      correlation += exponential; 
      searchLocation.theta = rng->gaussian(currentLocation.theta, correlation); 
    }
    else 
    {
      cout << "CPFAStatus: Uninformed Search" << endl;
      //select new position 50 cm from current location for random search
      if (first_waypoint)
      {
        first_waypoint = false;
        searchLocation.theta = currentLocation.theta + M_PI;
      }
      else
      {
        //select new heading from Gaussian distribution around current heading
        //searchLocation.theta = rng->gaussian(currentLocation.theta, 0.25);
        searchLocation.theta = rng->gaussian(currentLocation.theta, correlation);
      }
       
    }  

    searchLocation.x = currentLocation.x + (search_step_size * cos(searchLocation.theta));
    searchLocation.y = currentLocation.y + (search_step_size * sin(searchLocation.theta));
        
    result.wpts.waypoints.clear();
    cout<<"Search: wpts.waypoint"<<endl;
    result.wpts.waypoints.insert(result.wpts.waypoints.begin(), searchLocation);

    cout<<"search 2. result.wpts.waypoints size ="<<result.wpts.waypoints.size()<<endl;
    cout<<".x= "<<result.wpts.waypoints[0].x<<endl;
		
    
    return result;
  }

}

bool SearchController::ShouldInterrupt(){
	cout<<"search controller should interrupt..."<<endl;
  ProcessData();
  cout<<"false"<<endl;
  return false;
}

bool SearchController::HasWork() {
  return true;
}

void SearchController::setSearchType(bool informed_search) 
{
  this->informed_search = informed_search;

  if(informed_search)
  {
    informed_search_time = current_time;
  }
}

void SearchController::SetCurrentTimeInMilliSecs( long int time )
{
  current_time = time;
}
void SearchController::senseLocalResourceDensity(int num_tags)
{
  if(num_tags > local_resource_density){
    local_resource_density = num_tags;
  }

  cout << "ROSAdapter: targetHandler" << endl;
  cout << "local_resource_density: " << local_resource_density << endl;
  cout << endl;
}

/*bool SearchController::layPheromone() 
{
  double poisson = getPoissonCDF(rate_of_laying_pheromone);
  double random_num = rng->uniformReal(0, 1);
  cout<<"poisson="<<poisson<<endl;
  cout<<"random_num="<<random_num<<endl;
  
  if(poisson > random_num) {
    cout << "Laying a pheromone..." << endl;
    cout << endl;
    return true;
  } else {
    return false;
  }
}*/



//void SearchController::insertPheromone(const int center_id, const vector<Point> &pheromone_trail)
/*void SearchController::insertPheromone( const vector<Point> &pheromone_trail)
{
  // At this point in time we are not making a pheromone trail
  // the first index of the trail is the same position as the pheromone location
  Point new_location = pheromone_trail[0];
  //Pheromone pheromone(new_location, pheromone_trail, ros::Time::now(), rate_of_pheromone_decay);
  Pheromone pheromone(new_location, pheromone_trail, current_time, rate_of_pheromone_decay);

  pheromones.push_back(pheromone);
  //pheromones[center_id].push_back(pheromone);
  cout << "PheromoneStatus: pheromoneLocation x: " << pheromone_trail[0].x << " y: " << pheromone_trail[0].y << endl;
 
  cout << "================================================================" << endl;
}*/

CPFAState SearchController::GetCPFAState() 
{
  return cpfa_state;
}

void SearchController::SetCPFAState(CPFAState state) {
  // If true, rover failed to pickup block and is restarting search
  if(state == search_with_informed_walk || state == search_with_uninformed_walk){
    local_resource_density = 0;
    //informed_search_start_time = ros::Time::now();
    informed_search_start_time = current_time;
  }

  cpfa_state = state;
  result.cpfa_state = state;
}

/*Point SearchController::getTargetLocation()
{
  return target_location;
}
*/
/*
int SearchController::GetCenterIdx(){
	return center_idx;
	}
*/
void SearchController::SetCenterLocation(Point centerLocation) {
  
  float diffX = this->centerLocation.x - centerLocation.x;
  float diffY = this->centerLocation.y - centerLocation.y;
  this->centerLocation = centerLocation;
  
  if (!result.wpts.waypoints.empty())
  {
  result.wpts.waypoints.back().x -= diffX;
  result.wpts.waypoints.back().y -= diffY;
  }
  
}

void SearchController::SetSuccesfullPickup() {
succesfullPickup = true;
Reset(); //qilu remove waypoints, let rovers go to site fidelity
}

bool SearchController::giveUpSearching() 
{
  double random_num = rng->uniformReal(0, 1);
  bool give_up = false;

  if (probability_of_returning_to_nest > random_num)  
  {
    give_up = true;
  } 

  return give_up;
}

double SearchController::getPoissonCDF(const double lambda)
{
  double sumAccumulator       = 1.0;
  double factorialAccumulator = 1.0;

  for (size_t i = 1; i <= local_resource_density; i++) {
    factorialAccumulator *= i;
    sumAccumulator += pow(lambda, i) / factorialAccumulator;
  }

  return (exp(-lambda) * sumAccumulator);
}

Point SearchController::GetCurrentLocation(){
	return this->currentLocation;
	}


void SearchController::SetCurrentLocation(Point currentLocation) {
  this->currentLocation = currentLocation;
}

void SearchController::ProcessData() {
}

/*void SearchController::updatePheromoneList()
{
  //ros::Time time = ros::Time::now();
  //map<int, vector<Pheromone>> newPheromoneList;
  vector<Pheromone> newPheromoneList;

  for(int i = 0; i < pheromones.size(); i++) {
    Point pLoc = pheromones[i].getLocation();
    //pheromones[i].update(time);
    pheromones[i].update();
    
    if(pheromones[i].isActive()) {
      newPheromoneList.push_back(pheromones[i]);
    }
  }

  pheromones = newPheromoneList;
}*/

/*void SearchController::setPheromone()
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

      //SetTarget(pheromones[i].GetLocation());
      //target_location = pheromones[center_idx][i].getLocation();
      //target_location = pheromones[i].getLocation();
      //cout << "pheromoneLocation x: " << target_location.x << " y: " << target_location.y << endl;

      // TrailToFollow = pheromones[i].GetTrail();

      //If we pick a pheromone, break out of this loop. 
      break;
    }

    //We didn't pick a pheromone! Remove its weight from randomWeight. 
    randomWeight -= pheromones[i].getWeight();
  }
}*/

void SearchController::setObstacleAvoidance(bool turn_direction)
{
  avoided_obstacle = true;
  this->turn_direction = turn_direction;
}

bool SearchController::OutOfArena(Point location){
	double lower = -arena_size/2.0;
	double upper = arena_size/2.0;
	if(location.x -0.5 <= lower || location.y -0.5 <= lower || location.x +0.5 >= upper || location.y +0.5 >= upper){
		return true;
    }
	return false;
	}

