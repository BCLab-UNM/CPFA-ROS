#ifndef PHEROMONE_H
#define PHEROMONE_H

#include <vector>
#include "Point.h"
#include <cmath>
#include <iostream>
/*****
 * Implementation of the iAnt Pheromone object used by the iAnt CPFA. iAnts build and maintain a list of these pheromone waypoint objects to use during
 * the informed search component of the CPFA algorithm.
 *****/

using namespace std; 

class Pheromone 
{

public:

  Pheromone(const Point new_location,
            const std::vector<Point> new_trail,
            const long int new_time,
            const double new_decay_rate);
  ~Pheromone();

  void update(long int current_time);
  bool isActive();
  double getWeight();
  Point getLocation();
  std::vector<Point> getTrail();

  //void SetCurrentTimeInMilliSecs( long int time ) { current_time = time/1e3; update(current_time);}

private:

  Point location;
  std::vector<Point> trail;

  long int last_updated;
  double decay_rate;
  double weight;
  double threshold;

  long int current_time;

};

#endif
