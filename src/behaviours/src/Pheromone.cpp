
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
Pheromone::Pheromone(const Point new_location, 
                     const std::vector<Point> new_trail, 
                     const ros::Time new_time, 
                     const double new_decay_rate)
{
    /* required initializations */
  location    = new_location;
  trail       = new_trail;
	last_updated = new_time;
	decay_rate   = new_decay_rate;

    /* standardized initializations */
	weight      = 1.0;
	threshold   = 0.001;
}

/*****
 * The pheromones slowly decay and eventually become inactive. This simulates
 * the effect of a chemical pheromone trail that dissipates over time.
 *****/
void Pheromone::update(const ros::Time& time)
{
    /* pheromones experience exponential decay with time */
    weight *= exp(-decay_rate * (time.toSec() - last_updated.toSec()));
    last_updated = time;
}

/*****
 * Return the pheromone's location.
 *****/
Point Pheromone::getLocation()
{
    return location;
}

/*****
 * Return the trail between the pheromone and the nest.
 *****/
std::vector<Point> Pheromone::getTrail() 
{
    return trail;
}

/*****
 * Return the weight, or strength, of this pheromone.
 *****/
double Pheromone::getWeight()
{
	return weight;
}

/*****
 * Is the pheromone active and usable?
 * TRUE:  weight >  threshold : the pheromone is active
 * FALSE: weight <= threshold : the pheromone is not active
 *****/
bool Pheromone::isActive() 
{
	return (weight > threshold);
}
