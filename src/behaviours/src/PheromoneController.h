#ifndef PHEROMONECONTROLLER_H
#define PHEROMONECONTROLLER_H

#include <vector>
#include "Point.h"
#include "Result.h"
#include "Controller.h"
#include "Pheromone.h"
#include "Tag.h"


class PheromoneController : virtual Controller
{
public:

  PheromoneController();
  ~PheromoneController();

  void Reset() override;
  Result DoWork() override;
  bool ShouldInterrupt() override;
  bool HasWork() override;

  void setCurrentTimeInMilliSecs( long int time ) { current_time = time; }
  void setTargetPickedUp () { targetHeld = true; }
  void setCurrentLocation (Point location) { currnet_location = location; }

  void setTargetDate (std::vector<Tag> tags);


private:

  Result result;

  std::vector<Pheromone> pheromones;

  long int current_time;

  Point currnet_location;

  bool targetHeld = false;
  bool sense_local_density = false;

  long int time_searching;
  int target_search_time = 8;
  
  int resource_density = 0;

};

#endif
