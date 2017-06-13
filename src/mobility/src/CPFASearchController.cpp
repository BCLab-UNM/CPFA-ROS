#include <ros/ros.h>
#include "CPFASearchController.h"


CPFASearchController::CPFASearchController() {
    searchState = START;
    searchLocationType = RANDOM;
    rng = new random_numbers::RandomNumberGenerator();
}

/**
 * Continues search pattern after interruption. For example, avoiding the
 * center or collisions.
 */
geometry_msgs::Pose2D CPFASearchController::continueInterruptedSearch(geometry_msgs::Pose2D currentLocation, geometry_msgs::Pose2D oldGoalLocation) {
    geometry_msgs::Pose2D newGoalLocation;

    //remainingGoalDist avoids magic numbers by calculating the dist
    double remainingGoalDist = hypot(oldGoalLocation.x - currentLocation.x, oldGoalLocation.y - currentLocation.y);

    //this of course assumes random walk continuation. Change for diffrent search methods.
    newGoalLocation.theta = oldGoalLocation.theta;
    newGoalLocation.x = currentLocation.x + (0.50 * cos(oldGoalLocation.theta)); //(remainingGoalDist * cos(oldGoalLocation.theta));
    newGoalLocation.y = currentLocation.y + (0.50 * sin(oldGoalLocation.theta)); //(remainingGoalDist * sin(oldGoalLocation.theta));

    return newGoalLocation;
}

/**
 * This function decides which state function to call based on the value
 * of the searchState variable.
 */
        geometry_msgs::Pose2D CPFASearchController::CPFAStateMachine(geometry_msgs::Pose2D currentLocation, geometry_msgs::Pose2D centerLocation) {

                switch(searchState) {
                    case START:
                        start();
                    case SET_SEARCH_LOCATION:
                        setSearchLocation(currentLocation);
                        break;
                    case TRAVEL_TO_SEARCH_SITE:
                        travelToSearchSite();
                        break;
                    case SEARCH_WITH_UNINFORMED_WALK:
                        searchWithUninformedWalk();
                        break;
                    case SEARCH_WITH_INFORMED_WALK:
                        searchWithInformedWalk();
                        break;
                    case SENSE_LOCAL_RESOURCE_DENSITY:
                        break;
                        //senseLocalResourceDensity();
                }

        return targetLocation; // will place into Result struct later
}

void CPFASearchController::start() {
    maxTags = 10;
    probabilityOfSwitchingToSearching = 0.5;
    probabilityOfReturningToNest = 0.5;
    uninformedSearchVariation = 2*M_PI;
    rateOfInformedSearchDecay = exp(5);
    rateOfSiteFidelity = 10;
    rateOfLayingPheromone = 10;
    rateOfPheromoneDecay = exp(10);
    travelStepSize = 0.5;

    // set heading for random search
    targetLocation.theta = rng->uniformReal(0, 2 * M_PI);

    /* Will get from congfig file
       Need max distance from center
       maxDistanceFromNest = 6;*/

    searchState = SET_SEARCH_LOCATION;
}

void CPFASearchController::setSearchLocation(geometry_msgs::Pose2D currentLocation) {
    if(searchLocationType == SITE_FIDELITY){
        // TODO
        return;
    }

    if(searchLocationType == PHERMONE){
        //TODO
        return;
    }

    // searchLocationType == RANDOM
    targetLocation.x = currentLocation.x + travelStepSize*cos(targetLocation.theta);
    targetLocation.y = currentLocation.y + travelStepSize*sin(targetLocation.theta);

    searchState = TRAVEL_TO_SEARCH_SITE;
}

void CPFASearchController::travelToSearchSite() {}

void CPFASearchController::searchWithUninformedWalk() {}

void CPFASearchController::searchWithInformedWalk() {}

void CPFASearchController::senseLocalResourceDensity() {}

void CPFASearchController::returnToNest() {
    // if (return to nest with block)
    // BLAHBLAH

    // else if (give up search) then ....
    targetLocation.theta = rng->uniformReal(0, 2 * M_PI);
}
