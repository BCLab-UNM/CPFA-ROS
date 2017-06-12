#ifndef CPFA_SEARCH_CONTROLLER
#define CPFA_SEARCH_CONTROLLER

#include <geometry_msgs/Pose2D.h>
#include <random_numbers/random_numbers.h>
#include "Pheromone.h"

/**
 * This class implements the search control algorithm for the rovers. The code
 * here should be modified and enhanced to improve search performance.
 *
 * The search controller is implemented with the CPFA.
 * (Central Place Foraging Algorithm)
 */
class CPFASearchController {

  public:

    enum CPFAState
    {
        START,
        SET_SEARCH_LOCATION,
        TRAVEL_TO_SEARCH_SITE,
        SEARCH_WITH_UNINFORMED_WALK,
        SEARCH_WITH_INFORMED_WALK,
        SENSE_LOCAL_RESOURCE_DENSITY
    };

    enum SearchLocationType
    {
        SITE_FIDELITY,
        PHERMONE,
        RANDOM
    };

    CPFASearchController();

    /** functions for basic search for testing etc. **/
    // performs search pattern
    geometry_msgs::Pose2D search(geometry_msgs::Pose2D currentLocation);
    // continues search pattern after interruption
    geometry_msgs::Pose2D continueInterruptedSearch(geometry_msgs::Pose2D currentLocation, geometry_msgs::Pose2D oldGoalLocation);

    /** functions for CPFA search **/    
    // CPFA state machine search; will eventually replace search defined above
    geometry_msgs::Pose2D CPFAStateMachine(geometry_msgs::Pose2D currentLocation, geometry_msgs::Pose2D centerLocation);

  private:

    // CPFA state functions
    void start();
    void setSearchLocation(geometry_msgs::Pose2D currentLocation);
    void travelToSearchSite();
    void searchWithUninformedWalk();
    void searchWithInformedWalk();
    void senseLocalResourceDensity();
    void returnToNest();
    
    // CPFA Parameters
    double probabilityOfSwitchingToSearching;
    double probabilityOfReturningToNest;
    double uninformedSearchVariation;
    double rateOfInformedSearchDecay;
    double rateOfSiteFidelity;
    double rateOfLayingPheromone;
    double rateOfPheromoneDecay;

    double travelStepSize;
    CPFAState searchState;
    SearchLocationType searchLocationType;
    random_numbers::RandomNumberGenerator* rng;
    int localResourceDensity;
    int maxDistanceFromNest;
    geometry_msgs::Pose2D targetLocation;
    std::vector<Pheromone> pheromones;
};

#endif /* CPFA_SEARCH_CONTROLLER */
