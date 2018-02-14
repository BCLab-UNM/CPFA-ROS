#ifndef CPFA_PARAMETERS_H
#define CPFA_PARAMETERS_H

struct CPFAParameters {
  const float random_seed = -1;

  // CPFA Parameters according to beyond pheromones
  const float probability_of_switching_to_searching = 0.045;
  const float probability_of_returning_to_nest = 0.01;

  // In radians
  const float uninformed_search_variation = 0.4; //range [0, pi]

  // Lower causes it to decay slower
  const float rate_of_informed_search_decay = 0.1666;

  // Reduce these to increase probability
  const float rate_of_following_site_fidelity = 5; //original is 0.3; range [0, 20]; 0 -> 1; 20 -> 0
  const float rate_of_laying_pheromone = 5; //range [0, 20]; 0 -> 1; 20 -> 0
  const float rate_of_pheromone_decay = 0.025; //range [0, 1];
  
};

#endif

