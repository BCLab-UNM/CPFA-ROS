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
    START,
    SET_SEARCH_LOCATION,
    TRAVEL_TO_SEARCH_SITE,
    SEARCH_WITH_UNINFORMED_WALK,
    SEARCH_WITH_INFORMED_WALK,
    SENSE_LOCAL_RESOURCE_DENSITY,
    RETURN_TO_NEST
};

/**
 *  The SearchLocationType enum contains all possible travel states for the rover to be in
 */
enum SearchLocationType
{
    SITE_FIDELITY,
    PHEROMONE,
    RANDOM
};



/**
 * This class implements the search control algorithm for the rovers. The code
 * here should be modified and enhanced to improve search performance.
 *
 * The search controller is implemented with the CPFA.
 * (Central Place Foraging Algorithm)
 */

class CPFASearchController {

public:
    CPFASearchController();
    CPFASearchController(std::string publishedName);

    /** functions for CPFA search **/
    // continues search pattern after interruption
    geometry_msgs::Pose2D continueInterruptedSearch(geometry_msgs::Pose2D currentLocation, geometry_msgs::Pose2D oldGoalLocation, geometry_msgs::Pose2D centerLocation);

    // CPFA state machine search returns target locations for next step in CPFA
    geometry_msgs::Pose2D CPFAStateMachine(geometry_msgs::Pose2D currentLocation, geometry_msgs::Pose2D centerLocation);

    /* While in the SENSE_LOCAL_RESOURCE_DENSITY state, target handler
     * in mobility passes in number of resource tags seen by rover
     * while picking up a resource.
     *
     * CPFA Search Controller keeps track of maximum tags seen
     * by rover at the site location.
    */
    void senseLocalResourceDensity(int numTags);

    /* Calculates probability of laying pheromone depending on local
     * resource density and returns whether pheromone will be laid.
     * Called in mobility after successful pick-up of resource.
    */
    bool layPheromone();

    /*
     * Mobility passes in pheromone to CPFASearchController after resource is picked up.
    */
    void insertPheromone(std::vector<geometry_msgs::Pose2D> pheromoneTrail);


    CPFAState getState();
    void setState(CPFAState state);

    SearchLocationType getSearchLocationType();
    void setSearchLocationType(SearchLocationType type);

    geometry_msgs::Pose2D getTargetLocation();
    void setTargetLocation(geometry_msgs::Pose2D siteLocation, geometry_msgs::Pose2D centerLocation);

    void setArenaSize(int numRovers);

private:

    // CPFA state functions
    void start();
    void setSearchLocation(geometry_msgs::Pose2D currentLocation, geometry_msgs::Pose2D centerLocation);
    void travelToSearchSite(geometry_msgs::Pose2D currentLocation);
    void searchWithUninformedWalk(geometry_msgs::Pose2D currentLocation, geometry_msgs::Pose2D centerLocation);
    void searchWithInformedWalk(geometry_msgs::Pose2D currentLocation, geometry_msgs::Pose2D centerLocation);
    void returnToNest(geometry_msgs::Pose2D centerLocation);

    // CPFA Parameters
    double probabilityOfSwitchingToSearching;
    double probabilityOfReturningToNest;
    double uninformedSearchVariation;
    double rateOfInformedSearchDecay;
    double rateOfSiteFidelity;
    double rateOfLayingPheromone;
    double rateOfPheromoneDecay;

    // CPFA helper functions

    /* The distribution poisson describes the likelihood of finding at
     * least the quantity of resources determined by the c (localesourceDensity)
     * as parameterized by lambda. The robot returns to a previously found resource
     * location using site fidelity if the Poisson CDF, given the quantity of resources
     * determined by c (localResourceDensity), exceeds a uniform random value.
     *
     * POIS(c, lambda) > U(0, 1)
    */
    double getPoissonCDF(double lambda);

    // Returns the euclidian distance between two positions
    double distanceToLocation(geometry_msgs::Pose2D L1, geometry_msgs::Pose2D L2);

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
    bool giveUpSearching(geometry_msgs::Pose2D currentLocation, geometry_msgs::Pose2D centerLocation);

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
    void setPheromone(geometry_msgs::Pose2D centerLocation);

    CPFAState searchState;
    SearchLocationType searchLocationType;

    int arenaSize;
    int localResourceDensity; // An estimate of the density of the resources in the local region.
    double minDistanceToTarget; // Threshold to determine whether rover has reached a target location
    double travelStepSize; // Step size used for the rover travel state
    double searchStepSize; // Step size used while in either an informed or uninformed search state
    ros::Time informedSearchStartTime; // The time stamp for when the rover began the most recent informed search
    std::string roverName; // Used for debugging purposes to filter out a particular rover using ROS_INFO_STREAM
    geometry_msgs::Pose2D targetLocation; // Stores the nest goal location the rover will be traveling to
    std::vector<Pheromone> pheromones; // Stores all pheromone trails
    random_numbers::RandomNumberGenerator* rng;
};

#endif /* CPFA_SEARCH_CONTROLLER */
