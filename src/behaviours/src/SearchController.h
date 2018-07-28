#ifndef SEARCH_CONTROLLER
#define SEARCH_CONTROLLER

#include <random_numbers/random_numbers.h>
#include <vector>
#include <iostream>
#include "CPFAParameters.h"
#include "Pheromone.h"
#include "Controller.h"
#ifndef M_PI
#define M_PI 3.141592653589793238462643383279502884197169399375105820974944592307816406286
#endif
#ifndef ATTEMPT_MAX
#define ATTEMPT_MAX 8
#endif

/**
 * This class implements the search control algorithm for the rovers. The code
 * here should be modified and enhanced to improve search performance.
 */
class SearchController : virtual Controller {

public:

  SearchController();

  void Reset() override;

  // performs search pattern
  Result DoWork() override;
  bool ShouldInterrupt() override;
  bool HasWork() override;

  // sets the value of the current location
  //void UpdateData(geometry_msgs::Pose2D currentLocation, geometry_msgs::Pose2D centerLocation);
  void senseLocalResourceDensity(int num_tags);
  //bool layPheromone();
  //Point getTargetLocation();
  /*
   * Mobility passes in pheromone to CPFASearchController after resource is picked up.
   */
  //void insertPheromone(const int center_id, const std::vector<Point>& pheromone_trail);
  //void insertPheromone(const std::vector<Point>& pheromone_trail);
  CPFAState GetCPFAState() override;
  void SetCPFAState(CPFAState state) override;
  //int GetCenterIdx();
  //bool OutOfArena(Point location);
  //CPFASearchType GetCPFASearchType() override;
  //void SetCPFASearchType(CPFASearchType type) override;
  void SetCurrentLocation(Point currentLocation);
  Point GetCurrentLocation(); //qilu 12/2017
  
  void SetCenterLocation(Point centerLocation);
  void setObstacleAvoidance(bool turn_direction);
  void SetSuccesfullPickup();
  void SetCurrentTimeInMilliSecs( long int time );
  void setSearchType(bool informed_search);
  void SetArenaSize(int size);
  void SetRandomSearchLocation();
  bool ReachedWaypoint();
  void SetReachedWaypoint(bool reached);
  bool GiveupSearch();
  void SetGiveupSearch(bool giveup);
protected:

  void ProcessData();

private:
  
  // CPFA Parameters
  //double probability_of_switching_to_searching = 0.015;
  //double probability_of_returning_to_nest=0.001;
  //double uninformed_search_variation= 0.4;
  //double rate_of_informed_search_decay =0.1666;
  
  bool first_waypoint = true;
  /* The distribution poisson describes the likelihood of finding at
   * least the quantity of resources determined by the c (localesourceDensity)
   * as parameterized by lambda. The robot returns to a previously found resource
   * location using site fidelity if the Poisson CDF, given the quantity of resources
   * determined by c (localResourceDensity), exceeds a uniform random value.
   *
   * POIS(c, lambda) > U(0, 1)
   */
  double getPoissonCDF(const double lambda);
  bool giveUpSearching();
  /* Used every time a new pheromone is inserted into the CPFA Search Controller.
   *
   * The strength of the pheromone decays over time. Waypoints are removed once
   * their value drops below a certain threshold.
   */
  //void updatePheromoneList();
  /* While in the PHEROMONE state and after determining whether the rover should
   * use a pheromone, this function is called to determine which pheromone in our list
   * of pheromones should be selected as a target location and will provide a path
   * of waypoints to be followed to reach that location.
   */
  //void setPheromone();
  CPFAState cpfa_state = start_state;
  int arena_size = 14.0;
  int local_resource_density; // An estimate of the density of the resources in the local region.
  //std::vector<Pheromone> pheromones; // Stores all pheromone trails
  //map<int, vector<Pheromone>> pheromones;// qilu 08/2017
  bool succesfullPickup = false;
  bool avoided_obstacle = false;
  bool turn_direction = false;
  random_numbers::RandomNumberGenerator* rng;
  Point currentLocation;
  Point centerLocation;
  Point searchLocation;
  int attemptCount = 0;
  //struct for returning data to ROS adapter
  Result result;
  bool reachedWaypoint = false;
  //bool reachedCenter = false;
   bool giveupSearch = false;
   
  // Search state
  // Flag to allow special behaviour for the first waypoint
/*  bool succesfullPickup = false;
  bool informed_search = false;
  float search_step_size = 0.5;

  long int current_time = 0;
  long int informed_search_time = 0;
CPFAParameters CPFA_parameters;
  
*/
  CPFAParameters CPFA_parameters;
  bool informed_search = false;
  float search_step_size = 0.5;

  long int current_time = 0;
  long int informed_search_time = 0;
  long int informed_search_start_time =0;


};

#endif /* SEARCH_CONTROLLER */
