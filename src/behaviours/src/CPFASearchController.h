#ifndef CPFA_SEARCH_CONTROLLER
#define CPFA_SEARCH_CONTROLLER

// ROS Message Types
#include <ros/ros.h>
#include <random_numbers/random_numbers.h>

// Standard Libraries
#include <vector>
#include <iostream>

// Local messages and/or classes
#include "behaviours/Rover.h"
#include "Pheromone.h"
#include "Controller.h"
#include "Point.h"
#include "Result.h"

#include <string>

class CPFASearchController : virtual Controller {

public:

  CPFASearchController(std::string name);

  void Reset() override;

  // performs search pattern
  Result DoWork() override;
  bool ShouldInterrupt() override;
  bool HasWork() override;

  //=======================================================================
  /** functions for CPFA search **/

  /* While in the SENSE_LOCAL_RESOURCE_DENSITY state, target handler
   * in behaviours passes in number of resource tags seen by rover
   * while picking up a resource.
   *
   * CPFA Search Controller keeps track of maximum tags seen
   * by rover at the site location.
   */
  void senseLocalResourceDensity(int num_tags);

  /* Calculates probability of laying pheromone depending on local
   * resource density and returns whether pheromone will be laid.
   * Called in behaviours after successful pick-up of resource.
   */
  bool layPheromone();

  /*
   * Mobility passes in pheromone to CPFASearchController after resource is picked up.
   */
  void insertPheromone(const std::vector<Point>& pheromone_trail);

  CPFAState GetCPFAState() override;
  void SetCPFAState(CPFAState state) override;

  CPFASearchType GetCPFASearchType() override;
  void SetCPFASearchType(CPFASearchType type) override;

  Point getTargetLocation();
  void setTargetLocation(const Point& site_location);

  void SetSuccesfullPickup();
  void setArenaSize(int num_rovers);
  void SetCurrentLocation(Point currentLocation);
  void SetCenterLocation(Point centerLocation);

protected:

  void ProcessData();

private:

  //==============================================================================
  // CPFA state functions
  void start();
  void setSearchLocation();
  void travelToSearchSite();
  void searchWithUninformedWalk();
  void searchWithInformedWalk();
  void returnToNest();

  // CPFA Parameters
  double probability_of_switching_to_searching;
  double probability_of_returning_to_nest;
  double uninformed_search_variation;
  double rate_of_informed_search_decay;
  double rate_of_site_fidelity;
  double rate_of_laying_pheromone;
  double rate_of_pheromone_decay;

  // CPFA helper functions

  /* The distribution poisson describes the likelihood of finding at
   * least the quantity of resources determined by the c (localesourceDensity)
   * as parameterized by lambda. The robot returns to a previously found resource
   * location using site fidelity if the Poisson CDF, given the quantity of resources
   * determined by c (localResourceDensity), exceeds a uniform random value.
   *
   * POIS(c, lambda) > U(0, 1)
   */
  double getPoissonCDF(const double lambda);

  // Returns the euclidian distance between two positions
  double distanceToLocation(const Point& L1, const Point& L2);

  /* Used while in the SEARCH_WITH_INFORMED_WALK state
   *
   * If the robot is informed about the location of resources (via site fidelity or pheromones),
   * it searches using an informed correlated random walk where the standard deviation is defined
   * by the following equation as defined in this function.
   *
   * The standard deviation of the successive turning angles of the informed
   * random walk decays as a function of time t, producing an initially undirected randomized search
   * that becomes more correlated over time. This time decay allows the robot to search locally where it expects
   * to find a resource, but to straighten its path and disperse to another location if the resource is not
   * found.
   */
  double calculateInformedWalkCorrelation();

  /* Used while using an informed or uninformed search walk
   *
   * After the rover has traversed a predefined search step size
   * this function determines whether the rover should keep searching
   * or give up searching and return to the nest.
   */
  bool giveUpSearching(const Point& current_location,
      const Point& center_location);

  /* Used every time a new pheromone is inserted into the CPFA Search Controller.
   *
   * The strength of the pheromone decays over time. Waypoints are removed once
   * their value drops below a certain threshold.
   */
  void updatePheromoneList();

  /* While in the PHEROMONE state and after determining whether the rover should
   * use a pheromone, this function is called to determine which pheromone in our list
   * of pheromones should be selected as a target location and will provide a path
   * of waypoints to be followed to reach that location.
   */
  void setPheromone(const Point& center_location);

  CPFAState cpfa_state = start_state;
  CPFASearchType cpfa_search_type = random_search;

  int arena_size;
  int attempt_count = 0; 
  int local_resource_density; // An estimate of the density of the resources in the local region.
  bool succesfull_pickup = false;
  double min_distance_to_target; // Threshold to determine whether rover has reached a target location
  double travel_step_size; // Step size used for the rover travel state
  double search_step_size; // Step size used while in either an informed or uninformed search state
  ros::Time informed_search_start_time; // The time stamp for when the rover began the most recent informed search
  std::string rover_name; // Used for debugging purposes to filter out a particular rover using ROS_INFO_STREAM
  Point target_location; // Stores the nest goal location the rover will be traveling to
  Point current_location;
  Point center_location;

  std::vector<Pheromone> pheromones; // Stores all pheromone trails
  random_numbers::RandomNumberGenerator* rng;
  
  //struct for returning data to ROS adapter
  Result result;

};

#endif /* CPFA_SEARCH_CONTROLLER */
