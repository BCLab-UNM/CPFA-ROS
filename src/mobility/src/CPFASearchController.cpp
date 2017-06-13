#include "CPFASearchController.h"

CPFASearchController::CPFASearchController() {
    searchState = START;
    rng = new random_numbers::RandomNumberGenerator();
}

/**
 * This function decides which state function to call based on the value
 * of the searchState variable.
 */

geometry_msgs::Pose2D CPFASearchController::search(geometry_msgs::Pose2D currentLocation) {
    // Result myResult;

    switch(searchState) {
        case START:
            start();
            break;
        case SET_SEARCH_LOCATION:
            setSearchLocation();
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

    return searchLocation;
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

void CPFASearchController::start() {
    searchState = SET_SEARCH_LOCATION;
}

void CPFASearchController::setSearchLocation() {
}


void CPFASearchController::travelToSearchSite() {}

void CPFASearchController::searchWithUninformedWalk() {}

void CPFASearchController::searchWithInformedWalk() {}

void CPFASearchController::senseLocalResourceDensity() {}

void CPFASearchController::returnToNest() {}
