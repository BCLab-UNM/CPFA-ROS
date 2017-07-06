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
    updatePheromoneList();

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
    // At this point in time we are not making a pheromone trail
    // the first index of the trail is the same position as the pheromone location
    geometry_msgs::Pose2D newLocation = pheromoneTrail[0];
    Pheromone pheromone(newLocation, pheromoneTrail, ros::Time::now(), rateOfPheromoneDecay);

    pheromones.push_back(pheromone);
    ROS_INFO_STREAM(roverName << "CPFA: inserting pheromone pheromones.size(): " << pheromones.size());
    ROS_INFO_STREAM(roverName << "CPFA: pheromoneLocation x: " << pheromoneTrail[0].x << " y: " << pheromoneTrail[0].y);
}


CPFAState CPFASearchController::getState() {
    return searchState;
}

void CPFASearchController::setState(CPFAState state) {
    if(state == SET_SEARCH_LOCATION)
        ROS_INFO_STREAM(roverName << "CPFA: SET_SEARCH_LOCATION");
    else if(state == TRAVEL_TO_SEARCH_SITE)
        ROS_INFO_STREAM(roverName << "CPFA: TRAVEL_TO_SEARCH_SITE");
    else if(state == SEARCH_WITH_UNINFORMED_WALK)
        ROS_INFO_STREAM(roverName << "CPFA: SEARCH_WITH_UNINFORMED_WALK");
    else if(state == SEARCH_WITH_INFORMED_WALK)
        ROS_INFO_STREAM(roverName << "CPFA: SEARCH_WITH_INFORMED_WALK");
    else if(state == SENSE_LOCAL_RESOURCE_DENSITY)
        ROS_INFO_STREAM(roverName << "CPFA: SENSE_LOCAL_RESOURCE_DENSITY");
    else
        ROS_INFO_STREAM(roverName << "CPFA: RETURN_TO_NEST");

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
    if(type == SITE_FIDELITY)
        ROS_INFO_STREAM(roverName << "CPFA: searchLocationType SITE_FIDELITY");
    else if(type == PHEROMONE)
        ROS_INFO_STREAM(roverName << "CPFA: searchLocationType PHEROMONE");
    else
        ROS_INFO_STREAM(roverName << "CPFA: searchLocationType RANDOM");

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
    probabilityOfSwitchingToSearching = 0.13; // Increasing grows the probability
    probabilityOfReturningToNest = 0.0; // Increasing grows the probability
    uninformedSearchVariation = 0.4; // The change in heading using uninformed search
    rateOfInformedSearchDecay = 1.0/6.0; // Inverse of the expected time to find a resource
    rateOfSiteFidelity = 0.3; // Lower grows the probability
    rateOfLayingPheromone = 5; // Lower grows the probability
    rateOfPheromoneDecay = 1.0/40.0; // Lower means pheromone trail lasts longer

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
        setPheromone(centerLocation);

        // Adjust heading so rover headers to target
        targetLocation.theta = atan2(targetLocation.y - currentLocation.y, targetLocation.x - currentLocation.x);
        return;
    }

    searchLocationType = RANDOM;
    ROS_INFO_STREAM(roverName << "CPFA: SearchLocationType RANDOM");

    //// Direction of center relative to rover. Add buffer for angle ranges.
    //float centerLocationDirection = atan2(centerLocation.y - currentLocation.y, centerLocation.x - currentLocation.x) + 4 * M_PI;

    //float distanceToCenter = distanceToLocation(currentLocation, centerLocation);

    //// The angle range relative to the center for the rover to ignore on one side
    //float angleRange = atan2(1, distanceToCenter);
    
    //float upperBound = centerLocationDirection - angleRange + 2 * M_PI;
    //float lowerBound = centerLocationDirection + angleRange;

    //// set heading for random search
    //targetLocation.theta = rng->uniformReal(lowerBound, upperBound);

    //while(upperBound > 2 * M_PI) {
        //upperBound -= 2 * M_PI;
    //}

    //while(lowerBound > 2 * M_PI) {
        //lowerBound -= 2 * M_PI;
    //}

    //ROS_INFO_STREAM(roverName << "CPFA: lowerBound: " << lowerBound << " upperBound: " << upperBound);

    targetLocation.theta = rng->uniformReal(0, 2 * M_PI);
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
    if(pheromones.size() > 0) {
        searchLocationType = PHEROMONE;
        searchState = SET_SEARCH_LOCATION;
        return;
    }

    if(giveUpSearching(currentLocation, centerLocation))
        return;

    targetLocation.theta = rng->gaussian(currentLocation.theta, uninformedSearchVariation);
    targetLocation.x = currentLocation.x + searchStepSize*cos(targetLocation.theta);
    targetLocation.y = currentLocation.y + searchStepSize*sin(targetLocation.theta);
}

void CPFASearchController::searchWithInformedWalk(geometry_msgs::Pose2D currentLocation, geometry_msgs::Pose2D centerLocation) {
    if(giveUpSearching(currentLocation, centerLocation))
        return;

    double correlation = calculateInformedWalkCorrelation();


    // Round to the nearest 100th
    float roundCorrelation = roundf(correlation * 100)/100;
    ROS_INFO_STREAM(roverName << "CPFA: correlation: " << roundCorrelation);
    if(roundCorrelation == uninformedSearchVariation) {
        // Informed search has decayed into uninformed search
        ROS_INFO_STREAM(roverName << "CPFA: Informed search has decayed into uninformed search...");
        searchState = SEARCH_WITH_UNINFORMED_WALK;
        searchLocationType = RANDOM;
    }


    targetLocation.theta = rng->gaussian(currentLocation.theta, correlation); 
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
        geometry_msgs::Pose2D pLoc = pheromones[i].GetLocation();
        pheromones[i].Update(time);

        if(pheromones[i].IsActive()) {
            newPheromoneList.push_back(pheromones[i]);
        }
    }


    pheromones = newPheromoneList;
}

void CPFASearchController::setPheromone(geometry_msgs::Pose2D centerLocation)
{
    ROS_INFO_STREAM(roverName << "CPFA: Setting pheromone location...");;
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
            ROS_INFO_STREAM(roverName << "CPFA: pheromoneLocation x: " << targetLocation.x << " y: " << targetLocation.y);

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
