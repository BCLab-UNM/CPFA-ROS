#include "CPFASearchController.h"

CPFASearchController::CPFASearchController() {
  searchState = START;
  rng = new random_numbers::RandomNumberGenerator();
}

/**
 * This code implements a basic random walk search.
 */
geometry_msgs::Pose2D CPFASearchController::search(geometry_msgs::Pose2D currentLocation) {
  geometry_msgs::Pose2D goalLocation;

  //select new heading from Gaussian distribution around current heading
  goalLocation.theta = rng->gaussian(currentLocation.theta, 0.25);

  //select new position 50 cm from current location
  goalLocation.x = currentLocation.x + (0.5 * cos(goalLocation.theta));
  goalLocation.y = currentLocation.y + (0.5 * sin(goalLocation.theta));

  return goalLocation;
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
void CPFASearchController::search() {
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
            senseLocalResourceDensity();
    }
    
    // return myResult;
}

void CPFASearchController::start() {}

void CPFASearchController::setSearchLocation() {}

void CPFASearchController::travelToSearchSite() {}

void CPFASearchController::searchWithUninformedWalk() {}

void CPFASearchController::searchWithInformedWalk() {}

void CPFASearchController::senseLocalResourceDensity() {}

void CPFASearchController::returnToNest() {}
