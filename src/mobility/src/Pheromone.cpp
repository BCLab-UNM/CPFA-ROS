
#include "Pheromone.h"

/*****
 * The  pheromone needs to keep track of four things:
 *
 * [1] location of the waypoint
 * [2] a trail to the nest
 * [3] simulation time at creation
 * [4] pheromone rate of decay
 *
 * The remaining variables always start with default values.
 *****/
Pheromone::Pheromone(geometry_msgs::Pose2D newLocation, std::vector<geometry_msgs::Pose2D> newTrail, ros::Time newTime, double newDecayRate)
{
    /* required initializations */
	location    = newLocation;
    trail       = newTrail;
	lastUpdated = newTime;
	decayRate   = newDecayRate;

    /* standardized initializations */
	weight      = 1.0;
	threshold   = 0.001;
}

/*****
 * The pheromones slowly decay and eventually become inactive. This simulates
 * the effect of a chemical pheromone trail that dissipates over time.
 *****/
void Pheromone::Update(ros::Time time) {
    /* pheromones experience exponential decay with time */
    weight *= exp(-decayRate * (time.toSec() - lastUpdated.toSec()));
    lastUpdated = time;
}

/*****
 * Return the pheromone's location.
 *****/
geometry_msgs::Pose2D Pheromone::GetLocation() {
    return location;
}

/*****
 * Return the trail between the pheromone and the nest.
 *****/
std::vector<geometry_msgs::Pose2D> Pheromone::GetTrail() {
    return trail;
}

/*****
 * Return the weight, or strength, of this pheromone.
 *****/
double Pheromone::GetWeight() {
	return weight;
}

/*****
 * Is the pheromone active and usable?
 * TRUE:  weight >  threshold : the pheromone is active
 * FALSE: weight <= threshold : the pheromone is not active
 *****/
bool Pheromone::IsActive() {
	return (weight > threshold);
}
