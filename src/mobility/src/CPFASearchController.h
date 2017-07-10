#ifndef CPFA_SEARCH_CONTROLLER
#define CPFA_SEARCH_CONTROLLER

#include <geometry_msgs/Pose2D.h>
#include <random_numbers/random_numbers.h>
#include <vector>
#include "mobility/Rover.h"
#include "Pheromone.h"

/**
 * The CPFAState enum contains all possible CPFA States for rover to be in
 */

enum CPFAState
{
  start_state,
  set_search_location,
  travel_to_search_site,
  search_with_uninformed_walk,
  search_with_informed_walk,
  sense_local_resource_density,
  return_to_nest
};

/**
 *  The SearchLocationType enum contains all possible travel states for the rover to be in
 */
enum SearchLocationType
{
  site_fidelity,
  pheromone,
  random_search
};



/**
 * This class implements the search control algorithm for the rovers. The code
 * here should be modified and enhanced to improve search performance.
 *
 * The search controller is implemented with the CPFA.
 * (Central Place Foraging Algorithm)
 */

class CPFASearchController 
{

  public:
    CPFASearchController();
    CPFASearchController(std::string published_name);

    /** functions for CPFA search **/
    // continues search pattern after interruption
    geometry_msgs::Pose2D continueInterruptedSearch(const geometry_msgs::Pose2D& current_location, 
                                                    const geometry_msgs::Pose2D& old_goal_location, 
                                                    const geometry_msgs::Pose2D& center_location);

    // CPFA state machine search returns target locations for next step in CPFA
    geometry_msgs::Pose2D CPFAStateMachine(const geometry_msgs::Pose2D& current_location,
                                           const geometry_msgs::Pose2D& center_location);

    /* While in the SENSE_LOCAL_RESOURCE_DENSITY state, target handler
     * in mobility passes in number of resource tags seen by rover
     * while picking up a resource.
     *
     * CPFA Search Controller keeps track of maximum tags seen
     * by rover at the site location.
     */
    void senseLocalResourceDensity(const int& num_tags);

    /* Calculates probability of laying pheromone depending on local
     * resource density and returns whether pheromone will be laid.
     * Called in mobility after successful pick-up of resource.
     */
    bool layPheromone();

    /*
     * Mobility passes in pheromone to CPFASearchController after resource is picked up.
     */
    void insertPheromone(const std::vector<geometry_msgs::Pose2D>& pheromone_trail);


    CPFAState getState();
    void setState(const CPFAState& state);

    SearchLocationType getSearchLocationType();
    void setSearchLocationType(const SearchLocationType& type);

    geometry_msgs::Pose2D getTargetLocation();
    void setTargetLocation(const geometry_msgs::Pose2D& site_location,
                           const geometry_msgs::Pose2D& center_location);

    void setArenaSize(const int& num_rovers);

  private:

    // CPFA state functions
    void start();
    void setSearchLocation(const geometry_msgs::Pose2D& current_location, 
                           const geometry_msgs::Pose2D& center_location);
    void travelToSearchSite(const geometry_msgs::Pose2D& current_location);
    void searchWithUninformedWalk(const geometry_msgs::Pose2D& current_location,
                                  const  geometry_msgs::Pose2D& center_location);
    void searchWithInformedWalk(const geometry_msgs::Pose2D& current_location, 
                                const geometry_msgs::Pose2D& center_location);
    void returnToNest(const geometry_msgs::Pose2D& center_location);

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
    double distanceToLocation(const geometry_msgs::Pose2D& L1, const geometry_msgs::Pose2D& L2);

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
    bool giveUpSearching(const geometry_msgs::Pose2D& current_location,
                         const geometry_msgs::Pose2D& center_location);

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
    void setPheromone(const geometry_msgs::Pose2D& center_location);

    CPFAState search_state;
    SearchLocationType search_location_type;

    int arena_size;
    int local_resource_density; // An estimate of the density of the resources in the local region.
    double min_distance_to_target; // Threshold to determine whether rover has reached a target location
    double travel_step_size; // Step size used for the rover travel state
    double search_step_size; // Step size used while in either an informed or uninformed search state
    ros::Time informed_search_start_time; // The time stamp for when the rover began the most recent informed search
    std::string rover_name; // Used for debugging purposes to filter out a particular rover using ROS_INFO_STREAM
    geometry_msgs::Pose2D target_location; // Stores the nest goal location the rover will be traveling to
    std::vector<Pheromone> pheromones; // Stores all pheromone trails
    random_numbers::RandomNumberGenerator* rng;
};

#endif /* CPFA_SEARCH_CONTROLLER */
