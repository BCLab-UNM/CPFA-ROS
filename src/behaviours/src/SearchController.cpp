#include "SearchController.h"
#include <angles/angles.h>

using namespace std;

SearchController::SearchController() {
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
  result.reset = false;
  //attemptCount = 0;
  result.wpts.waypoints.clear();
  first_waypoint = true; //qilu 01/2018
  succesfullPickup = false;
  informed_search = false;
}
	
void SearchController::SetArenaSize(int size)
{
	arena_size = size;
	}
	
/**
 * This code implements a basic random walk search.
 */
Result SearchController::DoWork() {
 //cout<<"SwitchStatus: SearchController::DoWork(), size="<<result.wpts.waypoints.size()<<endl;
  //cout<<"search 1. result.wpts.waypoints size ="<<result.wpts.waypoints.size()<<endl;
  
  
  if (!result.wpts.waypoints.empty()) 
  {
	  
	cout<<"TestStatusCPFAStatus: SearchCTRL dowork(); waypoint is ["<<result.wpts.waypoints[0].x<<", "<<result.wpts.waypoints[0].y<<"]"<<endl;
    cout<<"TestStatusCPFAStatus: currentLocation=["<<currentLocation.x<<","<<currentLocation.y<<"]" <<endl;
    //cout<<"TestStatus: GetCPFAState()="<<GetCPFAState()<<endl;
    
    if(ReachedWaypoint())
    {
		cout<<"TestStatusSwitchStatus: SearchCTRL, reach waypoint..."<<endl;
	    if(GetCPFAState() == avoid_obstacle)
        {
			if(attemptCount<3)
			{
				attemptCount++;//count the times to approach the location. If the rover always see an obstacle, it should give up. 
		        cout<<"TestStatus: travel to the previous location before avoiding obstacles "<<attemptCount<<endl; 
		        SetCPFAState(reach_search_site);
		        return result;	
		    }
		    else
		    {
				cout<<"TestStatus: Give up to go to previous location"<<endl;
				result.wpts.waypoints.clear();	
                SetReachedWaypoint(false);
                attemptCount = 0;
		    }
	        	  	
        }
        else if(GetCPFAState() == return_to_nest)
        {
		  cout<<"TestStatus: Returning to nest..."<<endl;
	      //SetCPFAState(reached_nest);
	      //cout<<"TestStatusSwitchStatus: reached nest..."<<endl;
		  cout<<"TestStatus: start a new round ..."<<endl;
		  informed_search = true;
          first_waypoint = true;   
          result.type = behavior;
          SetCPFAState(start_state);
          result.b = FAILED; //giveup search and return to nest
          result.reset = true;
          result.wpts.waypoints.clear();	
          SetReachedWaypoint(false);
          attemptCount = 0;
		  return result;  
	    }
	    /*else if(GetCPFAState() == reached_nest)
	    {
		  cout<<"TestStatus: start a new round ..."<<endl;
		  informed_search = true;
          first_waypoint = true;   
          result.type = behavior;
          SetCPFAState(start_state);
          result.b = FAILED; //giveup search and return to nest
          result.reset = true;
          result.wpts.waypoints.clear();	
          SetReachedWaypoint(false);
          attemptCount = 0;
		  return result;
		}*/
	    /*else
	    {
		  first_waypoint = false;
	      result.wpts.waypoints.clear();	
          SetReachedWaypoint(false);
          attemptCount = 0;
        }*/
	    
    }
  }
  //cout <<"CPFAStatus: attemptCount="<<attemptCount<<endl;
  //if (attemptCount > 0 && attemptCount < 5) {
  //  attemptCount++;
 //cout<<"succesfullPickup="<<succesfullPickup<<endl;
  if (succesfullPickup) 
  {
    succesfullPickup = false;
    attemptCount = 1;
    return result;
  }
    //return result;
  //}
  //else if (attemptCount >= 5 || attemptCount == 0) {
    //attemptCount = 1;


    result.type = waypoint;
    //Point  searchLocation;

    
    float correlation = CPFA_parameters.uninformed_search_variation;
      
    if(GiveupSearch())
    {
		cout<<"TestStatus: 1. Return to nest..."<<endl;
		//searchLocation.theta = rng->gaussian(currentLocation.theta, correlation);
        searchLocation.x = this->centerLocation.x;
        searchLocation.y = this->centerLocation.y; 
        SetGiveupSearch(false);
	}
	else
	{
	    if (informed_search)
        {
          cout << "TestStatus: Informed Search" << endl;
	      float exponential = exp(-CPFA_parameters.rate_of_informed_search_decay * informed_search_time);
	      exponential *= (4 * M_PI - CPFA_parameters.uninformed_search_variation);
	      correlation += exponential; 
	      searchLocation.theta = rng->gaussian(currentLocation.theta, correlation); 
	      searchLocation.x = currentLocation.x + (search_step_size * cos(searchLocation.theta));
	      searchLocation.y = currentLocation.y + (search_step_size * sin(searchLocation.theta));
	      //result.cpfa_state = search_with_informed_walk;
	      SetCPFAState(search_with_informed_walk);
	     //cout<<"set state: result.cpfa_state ="<<result.cpfa_state <<endl;
	    }
	    else 
	    {
	      cout << "TestStatus: Uninformed Search" << endl;
	      //select new position 50 cm from current location for random search
	      if (first_waypoint)
	      {
			cout<<"TestStatusSwitchStatus: first waypoint..."<<endl;  
	        first_waypoint = false;
	        //searchLocation.theta = currentLocation.theta + M_PI;
	       //cout<<"CPFAStatus: currentLocation.theta="<<currentLocation.theta<<"  "<<rng->uniformReal(-M_PI/2, M_PI/2)<<endl;
	        searchLocation.theta = atan2(currentLocation.y - this->centerLocation.y, currentLocation.x - this->centerLocation.x);//this is the direction from center to the rover
	        
	        searchLocation.theta += rng->uniformReal(-M_PI/4, M_PI/4); //sample an angle in this range based on the direction from the center to the rover, so rover will not collide with the collection disk when it travels in this direction
	       //cout<<"CPFAStatus: searchLocation.theta="<<searchLocation.theta<<endl;
	        SetRandomSearchLocation();
	        //result.cpfa_state = travel_to_search_site;
	        SetCPFAState(travel_to_search_site);
	       //cout<<"SwitchStatus: set state: cpfa_state ="<<GetCPFAState() <<endl;
	        /*searchLocation.x = currentLocation.x + (arena_size * cos(searchLocation.theta));//need to be updated, qilu 
	        searchLocation.y = currentLocation.y + (10*search_step_size * sin(searchLocation.theta));//need to be updated, qilu 
	        */ 
	      }
	      else
	      {
	        //select new heading from Gaussian distribution around current heading
	        //searchLocation.theta = rng->gaussian(currentLocation.theta, 0.25);
	       //cout<<"SwitchStatus: not first waypoint..."<<endl;
	        searchLocation.theta = rng->gaussian(currentLocation.theta, correlation);
	        searchLocation.x = currentLocation.x + (search_step_size * cos(searchLocation.theta));
	        searchLocation.y = currentLocation.y + (search_step_size * sin(searchLocation.theta));
	        //result.cpfa_state = search_with_uninformed_walk;
	        SetCPFAState(search_with_uninformed_walk);
	       //cout<<"SwitchStatus: set state: result.cpfa_state ="<<result.cpfa_state <<endl;
	       //cout<<"set state: cpfa_state ="<<GetCPFAState() <<endl;
	      }
	       
	    //}  
	
	    //searchLocation.x = currentLocation.x + (search_step_size * cos(searchLocation.theta));
	    //searchLocation.y = currentLocation.y + (search_step_size * sin(searchLocation.theta));        
    
        }
    }
    result.wpts.waypoints.clear();
    result.wpts.waypoints.insert(result.wpts.waypoints.begin(), searchLocation);

   cout<<"TestStatusSwitchStatus: info/uninfo sampled wp=["<<searchLocation.x<<","<<searchLocation.y<<"]"<<endl;
		
    
    return result;
}

void SearchController::SetRandomSearchLocation() 
{	
	//cout<<"searchLocation.theta="<<searchLocation.theta<<endl;
	//float pos_x;
	if(cos(searchLocation.theta)<0)
	{
		searchLocation.x = rng->uniformReal(-arena_size*(3.0/4), -arena_size/2);
		//searchLocation.x = pos_x - currentLocation.x;
		//searchLocation.y = pos_x * tan(searchLocation.theta) - currentLocation.y;
		//cout<<"TestStatus...1. pos_x="<<searchLocation.x<<endl;
		}
    else
    {
		//searchLocation.x = arena_size/2 - 4*cos(searchLocation.theta);
		searchLocation.x = rng->uniformReal(arena_size*(3.0/4), arena_size/2);
		//cout<<"TestStatus...2. pos_x="<<searchLocation.x<<endl;
		}
	searchLocation.y = searchLocation.x * tan(searchLocation.theta);
    //cout<<"TestStatus: searchLocation.y="<<searchLocation.y<<endl;
    
	if(searchLocation.y >= arena_size/2) 
	{
		//cout<<"case 1..."<<endl;
		searchLocation.y = rng->uniformReal(arena_size/4, arena_size/2);
		searchLocation.x = searchLocation.y/tan(searchLocation.theta);
	}
	else if(searchLocation.y <= -arena_size/2)
	{
		//cout<<"case 2..."<<endl;
		searchLocation.y = rng->uniformReal(-arena_size/2, -arena_size/4);
		searchLocation.x = searchLocation.y/tan(searchLocation.theta);
		}	
	
	cout<<"TestStatus: searchLocation=["<<searchLocation.x<<", "<<searchLocation.y<<"]"<<endl;
	
	searchLocation.y += this->centerLocation.y;
    searchLocation.x += this->centerLocation.x;
  
	cout<<"TestStatus...relative searchLocation=["<<searchLocation.x<<", "<<searchLocation.y<<"]"<<endl;

}

bool SearchController::GiveupSearch()
{
	return giveupSearch;
	}

bool SearchController::ReachedWaypoint()
{
	return reachedWaypoint;
	}
  
  
void SearchController::SetGiveupSearch(bool giveup)
{
	giveupSearch= giveup;
	}


void SearchController::SetReachedWaypoint(bool reached)
{
	reachedWaypoint = reached;
	}
  
  

bool SearchController::ShouldInterrupt()
{
	//cout<<"SwitchStatus: search controller interrupt..."<<endl;
	//cout<<"TestStatusSwitchStatus: result.cpfa_state="<<result.cpfa_state<<endl;
	cout<<"SwitchStatus: cpfa_state="<<GetCPFAState()<<endl;
	//cout<<"SwitchStatus: time="<< current_time <<", "<<current_time % 30000 <<endl;
	if(current_time % 3000 <= 100) //check in every 30 seconds 
	{ 
		if(GetCPFAState() == travel_to_search_site)
		{
   		    //cout<<"SwitchStatus: time="<< current_time<<",  "<< current_time/1e3 <<", "<<current_time % 1000 <<endl;
	        double rndNum = rng->uniformReal(0.0, 1.0);
	       //cout<<"TestStatus: switch rndNum="<<rndNum<<endl;
	        //cout<<"SwitchStatus: switch rate="<<CPFA_parameters.probability_of_switching_to_searching<<endl;
	        if(rndNum < CPFA_parameters.probability_of_switching_to_searching)
	        {
			  first_waypoint = false;
			  result.reset = true;
              result.wpts.waypoints.clear();
   	          SetReachedWaypoint(true);
              //result.cpfa_state = search_with_uninformed_walk;
              SetCPFAState(search_with_uninformed_walk);
              cout<<"TestStatus:: Switch to search..."<<endl;
              return true;
            }
        }
        else if(GetCPFAState() == search_with_uninformed_walk || GetCPFAState() == search_with_informed_walk)
        {  
	        double rndNum = rng->uniformReal(0.0, 1.0);
	       //cout<<"TestStatus: giveup rndNum="<<rndNum<<endl;
	        if(rndNum < CPFA_parameters.probability_of_returning_to_nest)
	        {
				result.reset = true;
                result.wpts.waypoints.clear();
   	            SetGiveupSearch(true);
				//result.cpfa_state = return_to_nest;
				SetCPFAState(return_to_nest);
				cout<<"TestStatusSwitchStatus:: 2. Return to nest..."<<endl;
			    return true;	
				}
	    }
    }
    /*if(GetCPFAState() == reached_nest)
	{
		cout<<"TestStatusSwitchStatus: reached nest..."<<endl;
		first_waypoint = true; 
		informed_search = true;
		result.type = behavior;
        //SetCPFAState(start_state);
        result.b = FAILED; //giveup search and return to nest
        result.reset = true;
        return true;
	}*/
				
    ProcessData();
    //cout<<"SwitchStatus: false"<<endl;
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
 //cout<<"SearchCtrl: SetCPFAState ="<<result.cpfa_state <<endl;
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
	  //cout<<"TestStatus: SearchCTRL waypoint reset:["<<result.wpts.waypoints.back().x<<", "<<result.wpts.waypoints.back().y<<"]"<<endl;
  
  result.wpts.waypoints.back().x -= diffX;
  result.wpts.waypoints.back().y -= diffY;
   }
  
}

void SearchController::SetSuccesfullPickup() {
	//cout<<"Set succesfullPickup..."<<endl;
succesfullPickup = true;
Reset(); //qilu remove waypoints, let rovers go to site fidelity
}

bool SearchController::giveUpSearching() 
{
  double random_num = rng->uniformReal(0, 1);
  bool give_up = false;

  if (CPFA_parameters.probability_of_returning_to_nest > random_num)  
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

/*void SearchController::UpdatePheromoneList()
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

