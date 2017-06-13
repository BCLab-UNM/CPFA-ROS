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
        CPFASearchController();

        /** functions for CPFA search **/    
        // CPFA state machine search; will eventually replace search defined above
        geometry_msgs::Pose2D CPFAStateMachine(geometry_msgs::Pose2D currentLocation, geometry_msgs::Pose2D centerLocation);

        // continues search pattern after interruption
        geometry_msgs::Pose2D continueInterruptedSearch(geometry_msgs::Pose2D currentLocation, geometry_msgs::Pose2D oldGoalLocation);

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

        enum CPFAState
        {
            START,
            SET_SEARCH_LOCATION,
            TRAVEL_TO_SEARCH_SITE,
            SEARCH_WITH_UNINFORMED_WALK,
            SEARCH_WITH_INFORMED_WALK,
            SENSE_LOCAL_RESOURCE_DENSITY
        } searchState;

        enum SearchLocationType
        {
            SITE_FIDELITY,
            PHERMONE,
            RANDOM
        } searchLocationType;

        int maxTags;
        double travelStepSize;
        random_numbers::RandomNumberGenerator* rng;
        int localResourceDensity;
        int maxDistanceFromNest;
        geometry_msgs::Pose2D targetLocation;
        std::vector<Pheromone> pheromones;
};

#endif /* CPFA_SEARCH_CONTROLLER */
