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
    PHEROMONE,
    RANDOM
};

class CPFASearchController {

    public:
        CPFASearchController();

        /** functions for CPFA search **/
        // CPFA state machine search; will eventually replace search defined above
        geometry_msgs::Pose2D CPFAStateMachine(geometry_msgs::Pose2D currentLocation, geometry_msgs::Pose2D centerLocation);

        // continues search pattern after interruption
        geometry_msgs::Pose2D continueInterruptedSearch(geometry_msgs::Pose2D currentLocation, geometry_msgs::Pose2D oldGoalLocation);


    CPFAState getState();
    void setState(CPFAState state);

    SearchLocationType getSearchLocationType();
    void setSearchLocationType(SearchLocationType type);

private:

    // CPFA state functions
    void start();
    void setSearchLocation(geometry_msgs::Pose2D currentLocation);
    void travelToSearchSite(geometry_msgs::Pose2D currentLocation);
    void searchWithUninformedWalk(geometry_msgs::Pose2D currentLocation);
    void searchWithInformedWalk(geometry_msgs::Pose2D currentLocation);
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

    // CPFA helper functions
    double getPoissonCDF(double lambda);
    double distanceToLocation(geometry_msgs::Pose2D L1, geometry_msgs::Pose2D L2);
    double calculateInformedWalkCorrelation();

    ros::Time informedSearchStartTime;
    double minDistanceToTarget;
    CPFAState searchState;
    SearchLocationType searchLocationType;
    int maxTags;
    double travelStepSize;
    double searchStepSize;
    random_numbers::RandomNumberGenerator* rng;
    int localResourceDensity;
    int maxDistanceFromNest;
    geometry_msgs::Pose2D targetLocation;
    std::vector<Pheromone> pheromones;
};

#endif /* CPFA_SEARCH_CONTROLLER */
