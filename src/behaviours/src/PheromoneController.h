#ifndef PHEROMONECONTROLLER_H
#define PHEROMONECONTROLLER_H

#include <random_numbers/random_numbers.h>
#include <vector>
#include "Point.h"
#include "Result.h"
#include "Controller.h"
#include "Pheromone.h"
#include "Tag.h"
#include <angles/angles.h>
#ifndef ATTEMPT_MAX
#define ATTEMPT_MAX 8
#endif
class PheromoneController : virtual Controller
{
public:

  PheromoneController();
  ~PheromoneController();

  void Reset() override;
  Result DoWork() override;
  bool ShouldInterrupt() override;
  bool HasWork() override;
  bool SensingLocalDensity();
  bool SenseCompleted();

  void SetCenterLocation(Point centerLocation);
  void SetRoverInitLocation(Point location);
  void SetCurrentTimeInMilliSecs( long int time ) { current_time = time; }
  void SetTargetPickedUp() { targetHeld = true; }
  void SetCurrentLocation (Point location) { current_location = location; }

  void DriveToPheromoneTrail();
  void SetTagData(std::vector<Tag> tags);
  void SetCPFAState(CPFAState state) override;
  CPFAState GetCPFAState() override;
  //double getPoissonCDF(const double lambda);
  int GetResourceDensity();
  bool SelectPheromone();
  void InsertPheromone(const std::vector<Point>& pheromone_trail, double decay_rate);
  void UpdatePheromoneList();
  /* While in the PHEROMONE state and after determining whether the rover should
   * use a pheromone, this function is called to determine which pheromone in our list
   * of pheromones should be selected as a target location and will provide a path
   * of waypoints to be followed to reach that location.
   */
  
private:

  
  void ProcessData();
  random_numbers::RandomNumberGenerator* rng;
  
  CPFAState cpfa_state = start_state;
  Result result;

  std::vector<Pheromone> pheromones; // Stores all pheromone trails

  long int current_time;

  Point roverInitLocation;
  Point current_location;
  Point centerLocation;
  Point selected_pheromone;
  //bool targetHeld = false;
  bool targetHeld = true;//qilu 12/2017
  bool sense_local_density_completed = false;
  bool sense_local_resource_density = false; 
  bool drive_to_pheromone = false;

  //long int time_searching;
  //long int time_sensing =0;
  int const target_search_time = 8;
  float startAngle = 0;
  float rotateAngle = 0;
  float diffAngle = 0;
  float previousAngle = 0;
  float currentAngle = 0;
  std::vector<int> num_resource_detected;
  std::vector<float> detect_resource_angle;
  int resource_density = 0;
  int total_resource = 0;
  int attemptCount=0;

};

#endif
