#ifndef SITE_FIDELITY_CONTROLLER_H
#define SITE_FIDELITY_CONTROLLER_H

#include "Controller.h"
#include "Point.h"
#include "Tag.h"
#include "Result.h"

#include <math.h> // hypot

#ifndef ATTEMPT_MAX
#define ATTEMPT_MAX 8
#endif

class SiteFidelityController : virtual Controller {

public:
  SiteFidelityController();
  ~SiteFidelityController();

  void Reset() override;
  Result DoWork() override;
  bool ShouldInterrupt() override;
  bool HasWork() override;

  // Let site fidelity controller know that target 
  // has been picked up, so that we can save that current
  // location for site fidelity
  void SetTargetPickedUp(); 

  CPFAState cpfa_state = start_state;
  //struct for returning data to the ROS adapter
  Result result;
  void SetCurrentLocation(Point current_location);
  void SetCPFAState(CPFAState state) override;
  CPFAState GetCPFAState() override;
  bool SiteFidelityInvalid();
    
private:

    void ProcessData();
    void SiteFidelityReset();
    bool target_picked_up = false;
    Point site_fidelity_location;
    Point current_location;
    int attemptCount=0;
};
#endif // end header define
