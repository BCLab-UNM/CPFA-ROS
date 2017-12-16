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
  bool SensingLocalDensity();
  bool SenseCompleted();


  void SetCurrentTimeInMilliSecs( long int time ) { current_time = time; }
  void SetTargetPickedUp () { targetHeld = true; }
  void setCurrentLocation (Point location) { current_location = location; }

  void DriveToPheromoneTrail();
  void SetTagData(std::vector<Tag> tags);
  void SetCPFAState(CPFAState state) override;
  CPFAState GetCPFAState() override;
  //double getPoissonCDF(const double lambda);
  int GetResourceDensity();
  
private:

  void ProcessData();
  CPFAState cpfa_state = start_state;
  Result result;

  std::vector<Pheromone> pheromones;

  long int current_time;

  Point current_location;
  Point selected_pheromone;
  //bool targetHeld = false;
  bool targetHeld = true;//qilu 12/2017
  bool sense_local_density_completed = false;
  bool sense_local_resource_density = false; 
  bool drive_to_pheromone = false;

  long int time_searching;
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

};

#endif
