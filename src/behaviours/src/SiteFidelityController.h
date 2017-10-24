#ifndef SITE_FIDELITY_CONTROLLER_H
#define SITE_FIDELITY_CONTROLLER_H

#include "Controller.h"
#include "Point.h"
#include "Tag.h"
#include "Result.h"

#include <math.h> // hypot

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
  void setTargetPickedUp(); 

  void setCurrentLocation(Point current_location);

private:

    void ProcessData();

    bool target_picked_up = false;
    Point site_fidelity_location;
    Point current_location;
};
#endif // end header define
