#ifndef CPFA_PARAMETERS_H
#define CPFA_PARAMETERS_H

struct CPFAParameters {
  const float random_seed = -1;
  //For no obst.
  const float probability_of_switching_to_searching = 0.1;
  const float probability_of_returning_to_nest = 0.03;//0.01;
  // In radians
  const float uninformed_search_variation = 0.6; //range [0, pi]
  // Lower causes it to decay slower
  const float rate_of_informed_search_decay = 0.06;
  // Reduce these to increase probability
  const float rate_of_following_site_fidelity = 10; //0.3; original is 0.3; range [0, 20]; 0 -> 1; 20 -> 0 
  const float rate_of_laying_pheromone = 19; //range [0, 20]; 0 -> 1; 20 -> 0
  const float rate_of_pheromone_decay = 0.06; //range [0, 1]; Last only half min with 0.025, last 1 min and 10 sec with 0.01. 
  // more using site fidelity or pheromone wp causes high collision rate and reduces foraging rate. 
};

#endif

