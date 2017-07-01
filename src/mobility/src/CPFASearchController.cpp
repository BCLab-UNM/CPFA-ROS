#include <ros/ros.h>
#include "CPFASearchController.h"


// Public
CPFASearchController::CPFASearchController(){

}

CPFASearchController::CPFASearchController(std::string publishedName) {
    arenaSize = 15;
    travelStepSize = 0.5;
    searchStepSize = 0.5;
    minDistanceToTarget = 1;

    searchState = START;
    searchLocationType = RANDOM;
    rng = new random_numbers::RandomNumberGenerator();
    roverName = publishedName;
}

/**
 * Continues search pattern after interruption. For example, avoiding the
 * center or collisions.
 */
geometry_msgs::Pose2D CPFASearchController::continueInterruptedSearch(geometry_msgs::Pose2D currentLocation, geometry_msgs::Pose2D oldGoalLocation, geometry_msgs::Pose2D centerLocation) {

    // If the rover is near the edge of the arena, start searching
    if(searchState == TRAVEL_TO_SEARCH_SITE && distanceToLocation(currentLocation, centerLocation) > (arenaSize/2 - travelStepSize - 0.5)){
        ROS_INFO_STREAM(roverName << "CPFA: continueInterruptedSearch while traveling to search site");

        if(searchLocationType == SITE_FIDELITY || searchLocationType == PHEROMONE) {

            searchState = SEARCH_WITH_INFORMED_WALK;
            localResourceDensity = 0;
            informedSearchStartTime = ros::Time::now();
            ROS_INFO_STREAM(roverName << "CPFA: SEARCH_WITH_INFORMED_WALK");
        } else {

            searchState = SEARCH_WITH_UNINFORMED_WALK;
            ROS_INFO_STREAM(roverName << "CPFA: SEARCH_WITH_UNINFORMED_WALK");
        }

        return CPFAStateMachine(currentLocation, centerLocation);

    }else{
        geometry_msgs::Pose2D newGoalLocation;

        //remainingGoalDist avoids magic numbers by calculating the dist
        double remainingGoalDist = hypot(oldGoalLocation.x - currentLocation.x, oldGoalLocation.y - currentLocation.y);

        //this of course assumes random walk continuation. Change for diffrent search methods.
        newGoalLocation.theta = oldGoalLocation.theta;
        newGoalLocation.x = currentLocation.x + (0.50 * cos(oldGoalLocation.theta)); //(remainingGoalDist * cos(oldGoalLocation.theta));
        newGoalLocation.y = currentLocation.y + (0.50 * sin(oldGoalLocation.theta)); //(remainingGoalDist * sin(oldGoalLocation.theta));

        return newGoalLocation;
    }
}

/**
 * This function decides which state function to call based on the value
 * of the searchState variable.
 */
geometry_msgs::Pose2D CPFASearchController::CPFAStateMachine(geometry_msgs::Pose2D currentLocation, geometry_msgs::Pose2D centerLocation) {
    switch(searchState) {
    // Initializes CPFA parameters
    case START:
        start();

        // Sets target location to a site fidelity location, pheromone, or random location.
        // Preset when rover has picked up a resource and is returning to collection zone
    case SET_SEARCH_LOCATION:
        setSearchLocation(currentLocation, centerLocation);
        break;

        // Travels to target location while ignoring resources
    case TRAVEL_TO_SEARCH_SITE:
        travelToSearchSite(currentLocation);
        break;

        // Searches at a random target location
    case SEARCH_WITH_UNINFORMED_WALK:
        searchWithUninformedWalk(currentLocation, centerLocation);
        break;

        // Searches at a site fidelity location or pheromone location
    case SEARCH_WITH_INFORMED_WALK:
        searchWithInformedWalk(currentLocation, centerLocation);
        break;

        // Counts local resource density while picking up a resource
    case SENSE_LOCAL_RESOURCE_DENSITY:
        break;

        // Only set when Rover has given up search
    case RETURN_TO_NEST:
        returnToNest(currentLocation);
    }

    return targetLocation; // will place into Result struct later
}

void CPFASearchController::senseLocalResourceDensity(int numTags) {
    if(numTags > localResourceDensity){
        localResourceDensity = numTags;
        ROS_INFO_STREAM(roverName << "CPFA: local resource density: " << localResourceDensity);
    }
}

bool CPFASearchController::layPheromone() {
    double poisson = getPoissonCDF(rateOfLayingPheromone);
    double randomNum = rng->uniformReal(0, 1);

    ROS_INFO_STREAM(roverName << "CPFA: layPheromone, poisson: " << poisson << " > random number: " << randomNum);
    if(poisson > randomNum)
        return true;
    else
        return false;
}

void CPFASearchController::insertPheromone(std::vector<geometry_msgs::Pose2D> pheromoneTrail) {
    ROS_INFO_STREAM(roverName << "CPFA: inserting pheromone");
    // At this point in time we are not making a pheromone trail
    // the first index of the trail is the same position as the pheromone location
    geometry_msgs::Pose2D newLocation = pheromoneTrail[0];
    Pheromone pheromone(newLocation, pheromoneTrail, ros::Time::now(), rateOfPheromoneDecay);

    pheromones.push_back(pheromone);
}


CPFAState CPFASearchController::getState() {
    return searchState;
}

void CPFASearchController::setState(CPFAState state) {
    // If true, rover failed to pickup block and is restarting search
    if(state == SEARCH_WITH_INFORMED_WALK || state == SEARCH_WITH_UNINFORMED_WALK){
        localResourceDensity = 0;
        informedSearchStartTime = ros::Time::now();
    }

    searchState = state;
}

SearchLocationType CPFASearchController::getSearchLocationType() {
    return searchLocationType;
}

void CPFASearchController::setSearchLocationType(SearchLocationType type) {
    searchLocationType = type;
}

geometry_msgs::Pose2D CPFASearchController::getTargetLocation() {
    return targetLocation;
}

void CPFASearchController::setTargetLocation(geometry_msgs::Pose2D siteLocation, geometry_msgs::Pose2D centerLocation){
    targetLocation.theta = atan2(targetLocation.y - centerLocation.y, targetLocation.x - centerLocation.x);
    targetLocation.x = siteLocation.x;
    targetLocation.y = siteLocation.y;

    ROS_INFO_STREAM(roverName << "CPFA: setTargetLocation  targetLocation.x: " << targetLocation.x << " targetLocation.y: " << targetLocation.y);
}



// Private
void CPFASearchController::start() {
    ROS_INFO_STREAM(roverName << "CPFA: START");

    // CPFA parameters
    probabilityOfSwitchingToSearching = 0.10; // Increasing grows the probability
    probabilityOfReturningToNest = 0.0; // Increasing grows the probability
    uninformedSearchVariation = M_PI_2; // The change in heading using uninformed search
    rateOfInformedSearchDecay = 1.0/7.0; // Inverse of the expected time to find a resource
    rateOfSiteFidelity = 0.5; // Lower grows the probability
    rateOfLayingPheromone = 8; // Lower grows the probability
    rateOfPheromoneDecay = exp(10);

    searchState = SET_SEARCH_LOCATION;
    ROS_INFO_STREAM(roverName << "CPFA: SET_SEARCH_LOCATION");
}

void CPFASearchController::setSearchLocation(geometry_msgs::Pose2D currentLocation, geometry_msgs::Pose2D centerLocation) {
    searchState = TRAVEL_TO_SEARCH_SITE;
    ROS_INFO_STREAM(roverName << "CPFA: TRAVEL_TO_SEARCH_SITE");

    localResourceDensity = 0;

    if(searchLocationType == SITE_FIDELITY){
        double poisson = getPoissonCDF(rateOfSiteFidelity);
        double randomNum = rng->uniformReal(0, 1);

        ROS_INFO_STREAM(roverName << "CPFA: siteFidelity, poisson: " << poisson << " > random number: " << randomNum);
        if(poisson > randomNum) {
            ROS_INFO_STREAM(roverName << "CPFA: SearchLocationType SITE_FIDELITY");

            // Leave targetLocation to previous block location
            // Adjust heading so rover headers to target
            targetLocation.theta = atan2(targetLocation.y - currentLocation.y, targetLocation.x - currentLocation.x);
            return;
        } else {
            searchLocationType = PHEROMONE;
        }
    }

    if(searchLocationType == PHEROMONE && pheromones.size() > 0){
        ROS_INFO_STREAM(roverName << "CPFA: SearchLocationType PHEROMONE pheromones.size(): " << pheromones.size());
        updatePheromoneList();
        setPheromone(centerLocation);

        // Adjust heading so rover headers to target
        targetLocation.theta = atan2(targetLocation.y - currentLocation.y, targetLocation.x - currentLocation.x);
        return;
    }

    searchLocationType = RANDOM;
    ROS_INFO_STREAM(roverName << "CPFA: SearchLocationType RANDOM");

    // Direction of center relative to rover. Add buffer for angle ranges.
    float centerLocationDirection = atan2(centerLocation.y - currentLocation.y, centerLocation.x - currentLocation.x) + 4 * M_PI;

    float distanceToCenter = distanceToLocation(currentLocation, centerLocation);

    // The angle range relative to the center for the rover to ignore on one side
    float angleRange = atan2(1, distanceToCenter);
    
    float upperBound = centerLocationDirection - angleRange + 2 * M_PI;
    float lowerBound = centerLocationDirection + angleRange;

    // set heading for random search
    targetLocation.theta = rng->uniformReal(lowerBound, upperBound);

    while(upperBound > 2 * M_PI) {
        upperBound -= 2 * M_PI;
    }

    while(lowerBound > 2 * M_PI) {
        lowerBound -= 2 * M_PI;
    }

    ROS_INFO_STREAM(roverName << "CPFA: lowerBound: " << lowerBound << " upperBound: " << upperBound);

    travelToSearchSite(currentLocation);
}

void CPFASearchController::travelToSearchSite(geometry_msgs::Pose2D currentLocation) {

    if(searchLocationType == SITE_FIDELITY || searchLocationType == PHEROMONE) {
        // Reached site fidelity location and switch to informed search

        if(distanceToLocation(currentLocation, targetLocation) < minDistanceToTarget) {
            informedSearchStartTime = ros::Time::now();
            localResourceDensity = 0;
            searchState = SEARCH_WITH_INFORMED_WALK;
            ROS_INFO_STREAM(roverName << "CPFA: SEARCH_WITH_INFORMED_WALK");
        }


        // Heading to site fidelity, keep target location the same
        // Adjust heading so rover headers to target
        targetLocation.theta = atan2(targetLocation.y - currentLocation.y, targetLocation.x - currentLocation.x);
        return;
    }

    // searchLocationType is RANDOM
    if(rng->uniformReal(0, 1) < probabilityOfSwitchingToSearching){
        searchState = SEARCH_WITH_UNINFORMED_WALK;
        ROS_INFO_STREAM(roverName << "CPFA: SEARCH_WITH_UNINFORMED_WALK");
    }

    targetLocation.x = currentLocation.x + travelStepSize*cos(targetLocation.theta);
    targetLocation.y = currentLocation.y + travelStepSize*sin(targetLocation.theta);
}

void CPFASearchController::searchWithUninformedWalk(geometry_msgs::Pose2D currentLocation, geometry_msgs::Pose2D centerLocation) {
    if(giveUpSearching(currentLocation, centerLocation))
        return;

    double turnAngle = rng->gaussian(0, uninformedSearchVariation);
    targetLocation.theta = currentLocation.theta + turnAngle;
    targetLocation.x = currentLocation.x + searchStepSize*cos(targetLocation.theta);
    targetLocation.y = currentLocation.y + searchStepSize*sin(targetLocation.theta);
}

void CPFASearchController::searchWithInformedWalk(geometry_msgs::Pose2D currentLocation, geometry_msgs::Pose2D centerLocation) {
    if(giveUpSearching(currentLocation, centerLocation))
        return;

    double correlation = calculateInformedWalkCorrelation();
    ROS_INFO_STREAM(roverName << "CPFA: correlation: " << correlation);

    double turnAngle = rng->gaussian(0, correlation);
    targetLocation.theta = currentLocation.theta + turnAngle;
    targetLocation.x = currentLocation.x + searchStepSize*cos(targetLocation.theta);
    targetLocation.y = currentLocation.y + searchStepSize*sin(targetLocation.theta);
}

void CPFASearchController::returnToNest(geometry_msgs::Pose2D currentLocation) {
    if(distanceToLocation(currentLocation, targetLocation) < minDistanceToTarget) {
        searchState = SET_SEARCH_LOCATION;
        ROS_INFO_STREAM(roverName << "CPFA: SET_SEARCH_LOCATION");
    }
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
    ROS_INFO_STREAM(roverName << "CPFA: informedSearchTime: " << searchTime);

    return uninformedSearchVariation + (4 * M_PI - uninformedSearchVariation) * exp(-rateOfInformedSearchDecay * searchTime);
}

bool CPFASearchController::giveUpSearching(geometry_msgs::Pose2D currentLocation, geometry_msgs::Pose2D centerLocation) {
    double randomNum = rng->uniformReal(0, 1);

    if(probabilityOfReturningToNest > randomNum)  {
        searchLocationType = PHEROMONE;
        searchState = RETURN_TO_NEST;
        ROS_INFO_STREAM(roverName << "CPFA: RETURN_TO_NEST");

        // Sets the target location 1 meter radially out directly from the center of the nest
        targetLocation.theta = atan2(centerLocation.y - currentLocation.y, centerLocation.x - currentLocation.x);
        targetLocation.x = centerLocation.x - cos(targetLocation.theta);
        targetLocation.y = centerLocation.y - sin(targetLocation.theta);

        return true;
    }  else {
        return false;
    }
}

void CPFASearchController::updatePheromoneList(){
    ros::Time time = ros::Time::now();
    std::vector<Pheromone> newPheromoneList;

    for(int i = 0; i < pheromones.size(); i++) {

        pheromones[i].Update(time);

        if(pheromones[i].IsActive() == true) {
            newPheromoneList.push_back(pheromones[i]);
        }
    }

    pheromones = newPheromoneList;
}

void CPFASearchController::setPheromone(geometry_msgs::Pose2D centerLocation)
{
    double maxStrength = 0.0;
    double randomWeight = 0.0;

    /* Calculate a maximum strength based on active pheromone weights. */
    for(int i = 0; i < pheromones.size(); i++) {
        if(pheromones[i].IsActive()) {
            maxStrength += pheromones[i].GetWeight();
        }
    }

    /* Calculate a random weight. */
    randomWeight = rng->uniformReal(0.0, maxStrength);

    /* Randomly select an active pheromone to follow. */
    for(int i = 0; i < pheromones.size(); i++) {

        if(randomWeight < pheromones[i].GetWeight()) {
            /* We've chosen a pheromone! */

            //  SetTarget(pheromones[i].GetLocation());
            targetLocation = pheromones[i].GetLocation();

            //  TrailToFollow = pheromones[i].GetTrail();

            /* If we pick a pheromone, break out of this loop. */
            break;
        }

        /* We didn't pick a pheromone! Remove its weight from randomWeight. */
        randomWeight -= pheromones[i].GetWeight();
    }
}

void CPFASearchController::setArenaSize(int numRovers) {
    // Num rovers is every other rover, hence >= 3
    ROS_INFO_STREAM(roverName << "CPFA: numRovers: " << numRovers);
    if(numRovers > 3) {
        arenaSize = 22;
    }
}
