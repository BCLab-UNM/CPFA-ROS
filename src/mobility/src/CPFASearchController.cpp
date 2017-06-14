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
            travelToSearchSite(currentLocation);
            break;
        case SEARCH_WITH_UNINFORMED_WALK:
            searchWithUninformedWalk(currentLocation);
            break;
        case SEARCH_WITH_INFORMED_WALK:
            searchWithInformedWalk(currentLocation);
            break;
        case SENSE_LOCAL_RESOURCE_DENSITY:
            break;
            //senseLocalResourceDensity();
    }

    return targetLocation; // will place into Result struct later
}

void CPFASearchController::start() {
    ROS_INFO_STREAM("Inside START");
    maxTags = 10;
    probabilityOfSwitchingToSearching = 0.20;
    probabilityOfReturningToNest = 0.5;
    uninformedSearchVariation = 2*M_PI;
    rateOfInformedSearchDecay = exp(5);
    rateOfSiteFidelity = 10;
    rateOfLayingPheromone = 5;
    rateOfPheromoneDecay = exp(10);
    travelStepSize = 0.5;
    searchStepSize = 0.5;
    minDistanceToTarget = 0.50;

    /* Will get from congfig file
       Need max distance from center
       maxDistanceFromNest = 6;*/

    searchState = SET_SEARCH_LOCATION;
    ROS_INFO_STREAM("Inside SET_SEARCH_LOCATION");
}

void CPFASearchController::setSearchLocation(geometry_msgs::Pose2D currentLocation) {
    searchState = TRAVEL_TO_SEARCH_SITE;
    ROS_INFO_STREAM("Inside TRAVEL_TO_SEARCH_SITE");

    localResourceDensity = 0;

    if(searchLocationType == SITE_FIDELITY){
        double poisson = getPoissonCDF(rateOfSiteFidelity);
        double randomNum = rng->uniformReal(0, 1);

        if(poisson > randomNum) {
            // Leave targetLocation to previous block location
            ROS_INFO_STREAM("Inside Using site fidelity, poisson: " << poisson << " random number: " << randomNum);
            return;
        } else {
            searchLocationType = PHEROMONE;
        }
    }

    if(searchLocationType == PHEROMONE && pheromones.size() > 0){

        if(getPoissonCDF(rateOfLayingPheromone) > rng->uniformReal(0, 1)) {

            return;
        }
    }

    searchLocationType = RANDOM;

    // set heading for random search
    targetLocation.theta = rng->uniformReal(0, 2 * M_PI);
    travelToSearchSite(currentLocation);
}

void CPFASearchController::travelToSearchSite(geometry_msgs::Pose2D currentLocation) {


    if(searchLocationType == SITE_FIDELITY || searchLocationType == PHEROMONE) {        
        ROS_INFO_STREAM("Inside currentLoctaion.x: " << currentLocation.x << " currentLocation.y: " << currentLocation.y);
        ROS_INFO_STREAM("Inside targetLocation.x: " << targetLocation.x << " targetLocation.y: " << targetLocation.y);
        // Reached site fidelity location and switch to informed search
        if(distanceToLocation(currentLocation, targetLocation) < minDistanceToTarget) {
            informedSearchStartTime = ros::Time::now();
            searchState = SEARCH_WITH_INFORMED_WALK;
            ROS_INFO_STREAM("Inside SEARCH_WITH_INFORMED_WALK");
        }

        // Heading to site fidelity, keep target location the same
        return;
    }

    // searchLocationType is RANDOM
    if(rng->uniformReal(0, 1) < probabilityOfSwitchingToSearching){
        searchState = SEARCH_WITH_UNINFORMED_WALK;
        ROS_INFO_STREAM("Inside SEARCH_WITH_UNINFORMED_WALK");
    }

    targetLocation.x = currentLocation.x + travelStepSize*cos(targetLocation.theta);
    targetLocation.y = currentLocation.y + travelStepSize*sin(targetLocation.theta);
}

void CPFASearchController::searchWithUninformedWalk(geometry_msgs::Pose2D currentLocation) {
    double turnAngle = rng->gaussian(0, uninformedSearchVariation);
    targetLocation.theta = currentLocation.theta + turnAngle;
    targetLocation.x = currentLocation.x + searchStepSize*cos(targetLocation.theta);
    targetLocation.y = currentLocation.y + searchStepSize*sin(targetLocation.theta);
}

void CPFASearchController::searchWithInformedWalk(geometry_msgs::Pose2D currentLocation) {
    double correlation = calculateInformedWalkCorrelation();

    double turnAngle = rng->gaussian(0, correlation);
    targetLocation.theta = currentLocation.theta + turnAngle;
    targetLocation.x = currentLocation.x + searchStepSize*cos(targetLocation.theta);
    targetLocation.y = currentLocation.y + searchStepSize*sin(targetLocation.theta);
}

void CPFASearchController::senseLocalResourceDensity(int numTags) {
    if(numTags > localResourceDensity){
        localResourceDensity = numTags;
        ROS_INFO_STREAM("Inside local resource density: " << localResourceDensity);
    }
}

void CPFASearchController::returnToNest() {
    // if (return to nest with block)
    // BLAHBLAH

    // else if (give up search) then ....
    targetLocation.theta = rng->uniformReal(0, 2 * M_PI);
}

// Called by Mobility to determine whether Rover
// needs to travel uninterrupted (i.e. ignoring blocks)
CPFAState CPFASearchController::getState() {
    return searchState;
}

void CPFASearchController::setState(CPFAState state) {
    // If true, rover failed to pickup block and is restarting search
    if(state == SEARCH_WITH_INFORMED_WALK || state == SEARCH_WITH_UNINFORMED_WALK){
        localResourceDensity = 0;
    }
    searchState = state;
}

SearchLocationType CPFASearchController::getSearchLocationType() {
    return searchLocationType;
}

void CPFASearchController::setSearchLocationType(SearchLocationType type) {
    searchLocationType = type;
}

double CPFASearchController::getPoissonCDF(double lambda) {
    double sumAccumulator       = 1.0;
    double factorialAccumulator = 1.0;

    for (size_t i = 1; i <= localResourceDensity; i++) {
        factorialAccumulator *= i;
        sumAccumulator += pow(lambda, i) / factorialAccumulator;
    }

    return (exp(-lambda) * sumAccumulator);
}

double CPFASearchController::distanceToLocation(geometry_msgs::Pose2D L1, geometry_msgs::Pose2D L2) {
    return hypot(L1.x - L2.x, L1.y - L2.y);
}

double CPFASearchController::calculateInformedWalkCorrelation() {
    double searchTime = (ros::Time::now() - informedSearchStartTime).toSec();

    return uninformedSearchVariation + (4 * M_PI - uninformedSearchVariation) * exp(-rateOfInformedSearchDecay * searchTime);
}

void CPFASearchController::setTargetLocation(geometry_msgs::Pose2D siteLocation, geometry_msgs::Pose2D centerLocation){
    targetLocation = siteLocation;
    targetLocation.theta = atan2(targetLocation.y - centerLocation.y, targetLocation.x - centerLocation.x);
    ROS_INFO_STREAM("Inside setTargetLocation  targetLocation.x: " << targetLocation.x << " targetLocation.y: " << targetLocation.y);
}
